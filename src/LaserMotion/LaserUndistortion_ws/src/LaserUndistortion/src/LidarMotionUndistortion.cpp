#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include <LaserUndistortion/icp_odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

/**
 * @brief ROS adapter for the motion-compensation homework and ICP scaffold.
 *
 * There are only two algorithm implementation points in this package:
 *
 * 1. Lidar_MotionCalibration(): compensate each beam to the scan-start frame.
 * 2. PclIcpEstimator::estimate(): register the current local scan to the
 *    previous local scan.
 *
 * Everything else in this class deliberately handles data validation, TF,
 * coordinate-frame boundaries, publication, cache management, and diagnostics.
 */
class LidarMotionCalibrator
{
public:
    explicit LidarMotionCalibrator(tf::TransformListener& tf_listener)
        : tf_listener_(tf_listener), private_nh_("~")
    {
        private_nh_.param("scan_topic", scan_topic_, std::string("champion_scan"));
        private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
        private_nh_.param("laser_frame", laser_frame_, std::string("base_laser"));
        private_nh_.param("tf_timeout_sec", tf_timeout_sec_, 0.5);
        private_nh_.param("motion_segment_sec", motion_segment_sec_, 0.005);

        odom_frame_ = normalizeFrame(odom_frame_);
        laser_frame_ = normalizeFrame(laser_frame_);
        if (tf_timeout_sec_ <= 0.0 || !std::isfinite(tf_timeout_sec_))
        {
            ROS_WARN("Invalid ~tf_timeout_sec; using 0.5 seconds");
            tf_timeout_sec_ = 0.5;
        }
        if (motion_segment_sec_ <= 0.0 || !std::isfinite(motion_segment_sec_))
        {
            ROS_WARN("Invalid ~motion_segment_sec; using 0.005 seconds");
            motion_segment_sec_ = 0.005;
        }

        private_nh_.param("raw_cloud_topic", raw_cloud_topic_,
                          std::string("/laser_motion/raw_cloud"));
        private_nh_.param("corrected_cloud_topic", corrected_cloud_topic_,
                          std::string("/laser_motion/corrected_cloud"));
        private_nh_.param("corrected_local_cloud_topic", corrected_local_cloud_topic_,
                          std::string("/laser_motion/corrected_local_cloud"));
        private_nh_.param("icp_previous_cloud_topic", icp_previous_cloud_topic_,
                          std::string("/laser_motion/icp/previous_cloud"));
        private_nh_.param("icp_current_before_topic", icp_current_before_topic_,
                          std::string("/laser_motion/icp/current_before"));
        private_nh_.param("icp_current_aligned_topic", icp_current_aligned_topic_,
                          std::string("/laser_motion/icp/current_aligned"));

        const laser_undistortion::IcpConfig icp_config = loadIcpConfig();
        std::unique_ptr<laser_undistortion::IcpEstimator> estimator(
            new laser_undistortion::PclIcpEstimator(icp_config));
        icp_odometry_.reset(new laser_undistortion::IcpOdometry(
            std::move(estimator), icp_config));

        scan_sub_ = nh_.subscribe(scan_topic_, 10,
                                  &LidarMotionCalibrator::scanCallback, this);
        raw_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            raw_cloud_topic_, 10);
        corrected_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            corrected_cloud_topic_, 10);

        // These are diagnostics only.  The in-process ICP call below receives
        // local clouds directly; it never consumes an odom visualization cloud.
        corrected_local_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            corrected_local_cloud_topic_, 10);
        icp_previous_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            icp_previous_cloud_topic_, 10);
        icp_current_before_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            icp_current_before_topic_, 10);
        icp_current_aligned_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            icp_current_aligned_topic_, 10);

        ROS_INFO("Laser motion / ICP scaffold ready: motion and ICP solvers are TODOs");
    }

private:
    /**
     * @brief Receives a scan and drives the fixed pipeline around the two TODOs.
     */
    void scanCallback(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg)
    {
        if (!scan_msg)
        {
            ROS_WARN("Received a null laser scan pointer");
            return;
        }

        const champion_nav_msgs::ChampionNavLaserScan& scan = *scan_msg;
        // Every range/angle is measured in header.frame_id.  Reinterpreting a
        // scan from a different frame as laser_frame_ would create a geometrically
        // invalid cloud, so reject it until an explicit frame-conversion path exists.
        if (scan.header.frame_id.empty() ||
            normalizeFrame(scan.header.frame_id) != laser_frame_)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "Rejecting scan frame '%s': configured laser frame is '%s'",
                scan.header.frame_id.c_str(), laser_frame_.c_str());
            return;
        }

        if (scan.ranges.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max()))
        {
            ROS_WARN("Laser scan has too many beams");
            return;
        }

        const int beam_count = static_cast<int>(scan.ranges.size());
        if (beam_count == 0 || scan.angles.size() != scan.ranges.size())
        {
            ROS_WARN("Invalid laser scan: ranges and angles must be non-empty and equal-sized");
            return;
        }

        const double beam_time_increment = static_cast<double>(scan.time_increment);
        if (!std::isfinite(beam_time_increment) || beam_time_increment < 0.0)
        {
            ROS_WARN("Invalid laser scan time_increment");
            return;
        }

        const ros::Time start_time = scan.header.stamp;
        // Beam zero is at start_time, so the final beam is N - 1 increments later.
        const ros::Time end_time = start_time + ros::Duration(
            beam_time_increment * static_cast<double>(beam_count - 1));

        std::vector<double> ranges;
        std::vector<double> angles;
        ranges.reserve(static_cast<std::size_t>(beam_count));
        angles.reserve(static_cast<std::size_t>(beam_count));

        // Preserve beam ordering.  Index i represents both a geometric ray and
        // the timestamp start_time + i * time_increment.
        for (int i = 0; i < beam_count; ++i)
        {
            double range = static_cast<double>(scan.ranges[static_cast<std::size_t>(i)]);
            double angle = static_cast<double>(scan.angles[static_cast<std::size_t>(i)]);

            if (range < kMinimumRange || !std::isfinite(range))
            {
                range = 0.0;
            }
            if (!std::isfinite(angle))
            {
                ROS_WARN_THROTTLE(1.0, "Laser scan contains non-finite angles");
                range = 0.0;
                angle = 0.0;
            }

            ranges.push_back(range);
            angles.push_back(angle);
        }

        tf::Stamped<tf::Pose> frame_start_pose;
        if (!getLaserPose(frame_start_pose, start_time))
        {
            ROS_WARN("Start pose unavailable; skipping this scan");
            return;
        }

        // Build and publish the red reference before the calibration TODO changes
        // anything.  It is intentionally drawn as if all rays were acquired at t0.
        const laser_undistortion::PointCloudPtr raw_local_cloud =
            buildCloudInLaser(ranges, angles);
        publishCloudInOdom(*raw_local_cloud, asTransform(frame_start_pose),
                           start_time, raw_cloud_pub_);

        // Work on copies so a later TF failure cannot publish a partially
        // compensated green scan or feed a partial scan into ICP.
        std::vector<double> corrected_ranges = ranges;
        std::vector<double> corrected_angles = angles;
        if (!Lidar_Calibration(corrected_ranges, corrected_angles,
                               start_time, end_time, frame_start_pose))
        {
            ROS_WARN("Motion compensation failed; not publishing corrected/ICP clouds");
            return;
        }

        // This is the ICP input contract: all points are in the local laser frame
        // at this scan's start time.  The cloud is passed directly to ICP below.
        const laser_undistortion::PointCloudPtr corrected_local_cloud =
            buildCloudInLaser(corrected_ranges, corrected_angles);
        publishCloudInLaser(*corrected_local_cloud, start_time);
        publishCloudInOdom(*corrected_local_cloud, asTransform(frame_start_pose),
                           start_time, corrected_cloud_pub_);

        const laser_undistortion::IcpProcessResult icp_result =
            icp_odometry_->process(*corrected_local_cloud,
                                   asTransform(frame_start_pose), start_time);
        publishIcpDiagnostics(icp_result);

        if (icp_result.status != laser_undistortion::IcpProcessStatus::kFirstScanCached)
        {
            ROS_INFO_THROTTLE(1.0, "ICP scaffold status: %s (%s)",
                              laser_undistortion::toString(icp_result.status),
                              icp_result.message.c_str());
        }
    }

    /**
     * @brief Looks up T_odom_laser(t): a point in laser becomes a point in odom.
     */
    bool getLaserPose(tf::Stamped<tf::Pose>& odom_pose, const ros::Time& stamp) const
    {
        tf::Stamped<tf::Pose> laser_origin;
        laser_origin.setIdentity();
        laser_origin.frame_id_ = laser_frame_;
        laser_origin.stamp_ = stamp;

        try
        {
            if (!tf_listener_.waitForTransform(
                    odom_frame_, laser_frame_, stamp, ros::Duration(tf_timeout_sec_)))
            {
                ROS_ERROR("Timed out waiting for transform %s -> %s",
                          laser_frame_.c_str(), odom_frame_.c_str());
                return false;
            }
            tf_listener_.transformPose(odom_frame_, laser_origin, odom_pose);
        }
        catch (const tf::TransformException& exception)
        {
            ROS_ERROR("Cannot get laser pose: %s", exception.what());
            return false;
        }

        return true;
    }

    /**
     * @brief Student TODO #1: compensate one short scan segment.
     *
     * All corrected points must finish in the coordinate system represented by
     * frame_base_pose, which is fixed as the *whole scan's* first-beam pose.
     *
     * The target formula is:
     *   p_base = inverse(T_odom_laser(t0)) * T_odom_laser(tj) * p_laser
     *
     * A fully annotated implementation is in:
     * src/LaserMotion/doc/激光数据处理流程讲解.md
     */
    void Lidar_MotionCalibration(
        tf::Stamped<tf::Pose> frame_base_pose,
        tf::Stamped<tf::Pose> frame_start_pose,
        tf::Stamped<tf::Pose> frame_end_pose,
        std::vector<double>& ranges,
        std::vector<double>& angles,
        int start_index,
        int& beam_number)
    {
        (void)frame_base_pose;
        (void)frame_start_pose;
        (void)frame_end_pose;
        (void)ranges;
        (void)angles;
        (void)start_index;
        (void)beam_number;

        // TODO(MOTION): Hand-copy the documented implementation here.
        // Until this is implemented, corrected_ranges equals ranges, so the green
        // and red cloud intentionally overlap.  The surrounding scaffold remains
        // valid and lets you focus only on the compensation math.
    }

    /**
     * @brief Splits a scan into short non-overlapping segments and obtains TF.
     *
     * The function writes only its caller-provided copies.  On a TF failure it
     * returns false; the callback then refuses to publish a partial green cloud.
     */
    bool Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           const ros::Time& start_time,
                           const ros::Time& end_time,
                           const tf::Stamped<tf::Pose>& frame_base_pose)
    {
        if (ranges.empty() || ranges.size() != angles.size())
        {
            ROS_ERROR("ranges and angles must be non-empty and equal-sized");
            return false;
        }

        const std::size_t beam_count = ranges.size();
        const double time_increment = (beam_count <= 1)
            ? 0.0
            : (end_time - start_time).toSec() /
                  static_cast<double>(beam_count - 1);
        if (!std::isfinite(time_increment) || time_increment < 0.0)
        {
            ROS_ERROR("Invalid laser scan time interval");
            return false;
        }

        std::size_t segment_start_index = 0;
        tf::Stamped<tf::Pose> segment_start_pose = frame_base_pose;

        while (segment_start_index < beam_count)
        {
            std::size_t segment_end_index = segment_start_index;
            while (segment_end_index + 1 < beam_count &&
                   time_increment * static_cast<double>(
                       segment_end_index + 1 - segment_start_index) <=
                       motion_segment_sec_)
            {
                ++segment_end_index;
            }

            const ros::Time segment_end_time = start_time + ros::Duration(
                time_increment * static_cast<double>(segment_end_index));
            tf::Stamped<tf::Pose> segment_end_pose;
            if (segment_end_index == segment_start_index)
            {
                // A one-beam segment has the same start/end pose.  Avoid a
                // duplicate TF lookup and guarantee alpha=0 uses this pose.
                segment_end_pose = segment_start_pose;
            }
            else if (!getLaserPose(segment_end_pose, segment_end_time))
            {
                ROS_ERROR("Cannot get end pose for a motion-calibration segment");
                return false;
            }

            const std::size_t segment_count =
                segment_end_index - segment_start_index + 1;
            if (segment_start_index >
                    static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
                segment_count >
                    static_cast<std::size_t>(std::numeric_limits<int>::max()))
            {
                ROS_ERROR("Laser calibration segment exceeds int range");
                return false;
            }

            int mutable_segment_count = static_cast<int>(segment_count);
            Lidar_MotionCalibration(frame_base_pose,
                                    segment_start_pose,
                                    segment_end_pose,
                                    ranges,
                                    angles,
                                    static_cast<int>(segment_start_index),
                                    mutable_segment_count);

            const std::size_t next_segment_start_index = segment_end_index + 1;
            if (next_segment_start_index >= beam_count)
            {
                break;
            }

            // Segments must not overlap: every beam is changed at most once.
            segment_start_index = next_segment_start_index;
            const ros::Time next_segment_start_time = start_time + ros::Duration(
                time_increment * static_cast<double>(segment_start_index));
            if (!getLaserPose(segment_start_pose, next_segment_start_time))
            {
                ROS_ERROR("Cannot get start pose for a motion-calibration segment");
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Converts a valid polar scan into the scan-start laser coordinate axes.
     */
    laser_undistortion::PointCloudPtr buildCloudInLaser(
        const std::vector<double>& ranges, const std::vector<double>& angles) const
    {
        laser_undistortion::PointCloudPtr cloud(
            new laser_undistortion::PointCloud{});
        cloud->points.reserve(ranges.size());

        for (std::size_t index = 0; index < ranges.size(); ++index)
        {
            if (ranges[index] < kMinimumRange ||
                !std::isfinite(ranges[index]) || !std::isfinite(angles[index]))
            {
                continue;
            }

            const double x = ranges[index] * std::cos(angles[index]);
            const double y = ranges[index] * std::sin(angles[index]);
            if (!std::isfinite(x) || !std::isfinite(y))
            {
                continue;
            }

            pcl::PointXYZ point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = 0.0F;
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
            {
                continue;
            }
            cloud->points.push_back(point);
        }

        if (cloud->points.size() > std::numeric_limits<std::uint32_t>::max())
        {
            ROS_ERROR("Point cloud exceeds PCL width capacity");
            cloud->clear();
        }
        cloud->width = static_cast<std::uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }

    /**
     * @brief Publishes a local scan without applying odometry; useful for debugging
     * the exact cloud that is passed to the in-process ICP module.
     */
    void publishCloudInLaser(const laser_undistortion::PointCloud& local_cloud,
                             const ros::Time& stamp) const
    {
        laser_undistortion::PointCloud message = local_cloud;
        message.header.frame_id = laser_frame_;
        pcl_conversions::toPCL(stamp, message.header.stamp);
        corrected_local_cloud_pub_.publish(message);
    }

    /**
     * @brief Publishes a local cloud transformed exactly once into the odom frame.
     */
    void publishCloudInOdom(const laser_undistortion::PointCloud& local_cloud,
                            const tf::Transform& T_odom_laser,
                            const ros::Time& stamp,
                            const ros::Publisher& publisher) const
    {
        laser_undistortion::PointCloud message;
        message.header.frame_id = odom_frame_;
        pcl_conversions::toPCL(stamp, message.header.stamp);
        message.points.reserve(local_cloud.points.size());

        for (const pcl::PointXYZ& point : local_cloud.points)
        {
            const tf::Vector3 transformed =
                T_odom_laser * tf::Vector3(point.x, point.y, point.z);
            if (!std::isfinite(transformed.x()) ||
                !std::isfinite(transformed.y()) ||
                !std::isfinite(transformed.z()))
            {
                continue;
            }

            pcl::PointXYZ transformed_point;
            transformed_point.x = static_cast<float>(transformed.x());
            transformed_point.y = static_cast<float>(transformed.y());
            transformed_point.z = static_cast<float>(transformed.z());
            // A finite double can overflow when converted to float.  Do not mark
            // the cloud dense or hand downstream PCL code an infinite point.
            if (!std::isfinite(transformed_point.x) ||
                !std::isfinite(transformed_point.y) ||
                !std::isfinite(transformed_point.z))
            {
                continue;
            }
            message.points.push_back(transformed_point);
        }

        if (message.points.size() > std::numeric_limits<std::uint32_t>::max())
        {
            ROS_ERROR("Published point cloud exceeds PCL width capacity");
            return;
        }
        message.width = static_cast<std::uint32_t>(message.points.size());
        message.height = 1;
        message.is_dense = true;
        publisher.publish(message);
    }

    void publishIcpDiagnostics(
        const laser_undistortion::IcpProcessResult& result) const
    {
        if (result.previous_cloud_in_odom)
        {
            publishOdomDiagnostic(*result.previous_cloud_in_odom, result.previous_stamp,
                                  icp_previous_cloud_pub_);
        }
        if (result.current_before_in_odom)
        {
            publishOdomDiagnostic(*result.current_before_in_odom, result.current_stamp,
                                  icp_current_before_pub_);
        }
        // This stays unpublished until your ICP TODO produces a result that
        // passes every convergence, fitness, planar, and odom-prior gate.
        if (result.accepted && result.current_aligned_in_odom)
        {
            publishOdomDiagnostic(*result.current_aligned_in_odom, result.current_stamp,
                                  icp_current_aligned_pub_);
        }
    }

    void publishOdomDiagnostic(const laser_undistortion::PointCloud& cloud,
                               const ros::Time& stamp,
                               const ros::Publisher& publisher) const
    {
        laser_undistortion::PointCloud message = cloud;
        message.header.frame_id = odom_frame_;
        pcl_conversions::toPCL(stamp, message.header.stamp);
        publisher.publish(message);
    }

    laser_undistortion::IcpConfig loadIcpConfig()
    {
        laser_undistortion::IcpConfig config;
        private_nh_.param("icp/voxel_leaf_size", config.voxel_leaf_size, 0.05);
        private_nh_.param("icp/maximum_fitness", config.maximum_fitness, 0.10);
        private_nh_.param("icp/maximum_translation_correction",
                          config.maximum_translation_correction, 0.50);

        double maximum_yaw_correction_degrees{30.0};
        private_nh_.param("icp/maximum_yaw_correction_degrees",
                          maximum_yaw_correction_degrees, 30.0);

        int minimum_points{30};
        int minimum_correspondences{0};
        private_nh_.param("icp/minimum_points", minimum_points, 30);
        private_nh_.param("icp/minimum_correspondences", minimum_correspondences, 0);

        if (config.voxel_leaf_size < 0.0 || !std::isfinite(config.voxel_leaf_size))
        {
            ROS_WARN("Invalid ~icp/voxel_leaf_size; disabling voxel filtering");
            config.voxel_leaf_size = 0.0;
        }
        if (config.maximum_fitness <= 0.0 || !std::isfinite(config.maximum_fitness))
        {
            ROS_WARN("Invalid ~icp/maximum_fitness; using 0.10");
            config.maximum_fitness = 0.10;
        }
        if (config.maximum_translation_correction < 0.0 ||
            !std::isfinite(config.maximum_translation_correction))
        {
            ROS_WARN("Invalid ~icp/maximum_translation_correction; using 0.50");
            config.maximum_translation_correction = 0.50;
        }
        if (!std::isfinite(maximum_yaw_correction_degrees) ||
            maximum_yaw_correction_degrees < 0.0)
        {
            ROS_WARN("Invalid ~icp/maximum_yaw_correction_degrees; using 30 degrees");
            maximum_yaw_correction_degrees = 30.0;
        }

        config.minimum_points = static_cast<std::size_t>(std::max(minimum_points, 1));
        config.minimum_correspondences = static_cast<std::size_t>(
            std::max(minimum_correspondences, 0));
        config.maximum_yaw_correction = maximum_yaw_correction_degrees * kPi / 180.0;
        return config;
    }

    static tf::Transform asTransform(const tf::Stamped<tf::Pose>& pose)
    {
        return tf::Transform(pose.getRotation(), pose.getOrigin());
    }

    static std::string normalizeFrame(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/')
        {
            frame.erase(frame.begin());
        }
        return frame;
    }

    static constexpr double kMinimumRange{0.05};
    static constexpr double kPi{3.14159265358979323846};

    tf::TransformListener& tf_listener_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber scan_sub_;

    ros::Publisher raw_cloud_pub_;
    ros::Publisher corrected_cloud_pub_;
    ros::Publisher corrected_local_cloud_pub_;
    ros::Publisher icp_previous_cloud_pub_;
    ros::Publisher icp_current_before_pub_;
    ros::Publisher icp_current_aligned_pub_;

    std::unique_ptr<laser_undistortion::IcpOdometry> icp_odometry_;

    std::string scan_topic_;
    std::string odom_frame_;
    std::string laser_frame_;
    std::string raw_cloud_topic_;
    std::string corrected_cloud_topic_;
    std::string corrected_local_cloud_topic_;
    std::string icp_previous_cloud_topic_;
    std::string icp_current_before_topic_;
    std::string icp_current_aligned_topic_;
    double tf_timeout_sec_{0.5};
    double motion_segment_sec_{0.005};
};

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LidarMotionCalib");

    tf::TransformListener tf_listener(ros::Duration(10.0));
    LidarMotionCalibrator calibrator(tf_listener);

    ros::spin();
    return 0;
}
