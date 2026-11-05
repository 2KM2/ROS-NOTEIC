#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator(tf::TransformListener* tf)
    {
        tf_ = tf;
        scan_sub_ = nh_.subscribe("champion_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
        raw_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            "/laser_motion/raw_cloud", 10);
        corrected_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            "/laser_motion/corrected_cloud", 10);
    }


    // The TransformListener is owned by main(), not by this node.
    ~LidarMotionCalibrator() = default;

    // 拿到原始的激光数据来进行处理
    void ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg)
    {
        //转换到矫正需要的数据
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        champion_nav_msgs::ChampionNavLaserScan laserScanMsg = *scan_msg;

        if (laserScanMsg.ranges.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max()))
        {
            ROS_WARN("Laser scan has too many beams");
            return;
        }

        const int beamNum = static_cast<int>(laserScanMsg.ranges.size());
        if (beamNum == 0 || laserScanMsg.angles.size() != laserScanMsg.ranges.size())
        {
            ROS_WARN("Invalid laser scan data");
            return;
        }

        // The first beam is stamped at startTime; the final beam is N - 1
        // increments later.
        endTime = startTime + ros::Duration(
            laserScanMsg.time_increment * static_cast<double>(beamNum - 1));

        // Keep the message's chronological beam order so each index maps to the
        // matching acquisition time and TF pose.
        std::vector<double> angles;
        std::vector<double> ranges;
        angles.reserve(beamNum);
        ranges.reserve(beamNum);
        for (int i = 0; i < beamNum; ++i)
        {
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = laserScanMsg.angles[i];

            if (lidar_dist < 0.05 || !std::isfinite(lidar_dist))
                lidar_dist = 0.0;

            if (!std::isfinite(lidar_angle))
            {
                ROS_WARN_THROTTLE(1.0, "Laser scan contains non-finite angles");
                lidar_dist = 0.0;
                lidar_angle = 0.0;
            }

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }

        tf::Stamped<tf::Pose> frame_base_pose;
        if (!getLaserPose(frame_base_pose, startTime, tf_))
        {
            ROS_WARN("Not Start Pose, Can not calibrate");
            return;
        }

        // Keep a pre-calibration copy for RViz comparison, then perform the
        // homework correction in place and publish both clouds in odom.
        const std::vector<double> raw_ranges = ranges;
        const std::vector<double> raw_angles = angles;
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);

        PublishCloudInOdom(
            raw_ranges, raw_angles, frame_base_pose, startTime, raw_cloud_pub_);

        // TODO remains intentionally empty in the starter homework. Until it
        // is implemented, this cloud matches the red raw cloud.
        PublishCloudInOdom(
            ranges, angles, frame_base_pose, startTime, corrected_cloud_pub_);
    }


    void PublishCloudInOdom(const std::vector<double>& ranges,
                            const std::vector<double>& angles,
                            const tf::Stamped<tf::Pose>& frame_base_pose,
                            const ros::Time& stamp,
                            ros::Publisher& publisher)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.header.frame_id = "odom";
        pcl_conversions::toPCL(stamp, cloud.header.stamp);
        cloud.points.reserve(ranges.size());

        const double yaw = tf::getYaw(frame_base_pose.getRotation());
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        const double origin_x = frame_base_pose.getOrigin().x();
        const double origin_y = frame_base_pose.getOrigin().y();

        for (std::size_t i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            const double laser_x = ranges[i] * std::cos(angles[i]);
            const double laser_y = ranges[i] * std::sin(angles[i]);
            const double odom_x = laser_x * cos_yaw - laser_y * sin_yaw + origin_x;
            const double odom_y = laser_x * sin_yaw + laser_y * cos_yaw + origin_y;
            if (!std::isfinite(odom_x) || !std::isfinite(odom_y))
                continue;

            pcl::PointXYZ point;
            point.x = static_cast<float>(odom_x);
            point.y = static_cast<float>(odom_y);
            point.z = 0.0F;
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
                continue;

            cloud.points.push_back(point);
        }

        cloud.width = static_cast<std::uint32_t>(cloud.points.size());
        cloud.height = 1;
        cloud.is_dense = true;
        publisher.publish(cloud);
    }

    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_laser";
        robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

        // get the global pose of the robot
        try
        {
            if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        激光雷达运动畸变去除分段函数;
     *        在此分段函数中，认为机器人是匀速运动；
     * @param frame_base_pose       标定完毕之后的基准坐标系
     * @param frame_start_pose      本分段第一个激光点对应的位姿
     * @param frame_end_pose        本分段最后一个激光点对应的位姿
     * @param ranges                激光数据－－距离
     * @param angles                激光数据－－角度
     * @param startIndex            本分段第一个激光点在激光帧中的下标
     * @param beam_number           本分段的激光点数量
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
       //TODO
       //end of TODO
    }



    //激光雷达数据　分段线性进行插值　分段的周期为10ms
    //这里会调用Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
     * @param ranges 激光束的距离值集合
     * @param angle　激光束的角度值集合
     * @param startTime　第一束激光的时间戳
     * @param endTime　最后一束激光的时间戳
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //统计激光束的数量
        const int beamNumber = static_cast<int>(ranges.size());
        if (beamNumber == 0 || beamNumber != static_cast<int>(angles.size()))
        {
            ROS_ERROR("Error: ranges not match to the angles");
            return;
        }

        // 5ms来进行分段
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;

        //起始时间 us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        const double time_inc = (beamNumber <= 1)
            ? 0.0
            : (end_time - start_time) / static_cast<double>(beamNumber - 1);

        //当前插值的段的起始坐标
        int start_index = 0;

        //起始点的位姿 这里要得到起始点位置的原因是　起始点就是我们的base_pose
        //所有的激光点的基准位姿都会改成我们的base_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        int cnt = 0;
        // 基准坐标始终是整帧第一束激光对应的位姿。
        frame_base_pose = frame_start_pose;
        while (start_index < beamNumber)
        {
            int end_index = start_index;
            while (end_index + 1 < beamNumber &&
                   time_inc * static_cast<double>(end_index + 1 - start_index) <=
                       interpolation_time_duration)
            {
                ++end_index;
            }

            const double segment_end_time = start_time +
                time_inc * static_cast<double>(end_index - start_index);
            if (!getLaserPose(
                    frame_mid_pose, ros::Time(segment_end_time / 1000000.0), tf_))
            {
                ROS_ERROR("Segment %d end pose error", ++cnt);
                return;
            }

            int interp_count = end_index - start_index + 1;
            Lidar_MotionCalibration(frame_base_pose,
                                    frame_start_pose,
                                    frame_mid_pose,
                                    ranges,
                                    angles,
                                    start_index,
                                    interp_count);

            const int next_start_index = end_index + 1;
            if (next_start_index >= beamNumber)
                break;

            start_time = start_time +
                time_inc * static_cast<double>(next_start_index - start_index);
            start_index = next_start_index;
            if (!getLaserPose(
                    frame_start_pose, ros::Time(start_time / 1000000.0), tf_))
            {
                ROS_ERROR("Segment %d start pose error", ++cnt);
                return;
            }
        }
    }

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher raw_cloud_pub_;
    ros::Publisher corrected_cloud_pub_;
};




int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}
