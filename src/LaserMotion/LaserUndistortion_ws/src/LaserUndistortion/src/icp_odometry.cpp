#include <LaserUndistortion/icp_odometry.hpp>

#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_datatypes.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace laser_undistortion
{
namespace
{

bool isFinite(const double value)
{
    return std::isfinite(value);
}

bool isFiniteTransform(const tf::Transform& transform)
{
    const tf::Vector3& origin = transform.getOrigin();
    const tf::Quaternion& rotation = transform.getRotation();
    const double rotation_length_squared = rotation.length2();

    // A non-unit quaternion would produce a non-rigid matrix in Bullet/TF.
    constexpr double kQuaternionTolerance{1e-3};
    return isFinite(origin.x()) && isFinite(origin.y()) && isFinite(origin.z()) &&
           isFinite(rotation.x()) && isFinite(rotation.y()) &&
           isFinite(rotation.z()) && isFinite(rotation.w()) &&
           isFinite(rotation_length_squared) &&
           std::abs(rotation_length_squared - 1.0) <= kQuaternionTolerance;
}

bool isLevelSensorPose(const tf::Transform& transform)
{
    if (!isFiniteTransform(transform))
    {
        return false;
    }

    constexpr double kPlanarTolerance{1e-4};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    // A fixed mounting height is normal, so z is allowed here.  This framework
    // supports a 2D level scanner only: a pitched/rolled laser would make its
    // scan plane rotate in 3D and cannot be represented by the SE(2) ICP TODO.
    return std::abs(roll) <= kPlanarTolerance &&
           std::abs(pitch) <= kPlanarTolerance && isFinite(yaw);
}

bool isPlanarTransform(const tf::Transform& transform)
{
    constexpr double kPlanarTolerance{1e-4};
    return isLevelSensorPose(transform) &&
           std::abs(transform.getOrigin().z()) <= kPlanarTolerance;
}

PointCloudPtr compactFiniteCloud(const PointCloud& input)
{
    PointCloudPtr output(new PointCloud{});
    output->points.reserve(input.points.size());

    for (const pcl::PointXYZ& point : input.points)
    {
        if (pcl::isFinite(point))
        {
            output->points.push_back(point);
        }
    }

    if (output->points.size() > std::numeric_limits<std::uint32_t>::max())
    {
        return PointCloudPtr(new PointCloud{});
    }
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

PointCloudPtr transformCloud(const PointCloud& input, const tf::Transform& transform)
{
    PointCloudPtr output(new PointCloud{});
    output->points.reserve(input.points.size());

    for (const pcl::PointXYZ& point : input.points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const tf::Vector3 transformed = transform * tf::Vector3(point.x, point.y, point.z);
        if (!isFinite(transformed.x()) || !isFinite(transformed.y()) ||
            !isFinite(transformed.z()))
        {
            continue;
        }

        pcl::PointXYZ transformed_point;
        transformed_point.x = static_cast<float>(transformed.x());
        transformed_point.y = static_cast<float>(transformed.y());
        transformed_point.z = static_cast<float>(transformed.z());
        if (!pcl::isFinite(transformed_point))
        {
            continue;
        }
        output->points.push_back(transformed_point);
    }

    if (output->points.size() > std::numeric_limits<std::uint32_t>::max())
    {
        return PointCloudPtr(new PointCloud{});
    }
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

}  // namespace

PclIcpEstimator::PclIcpEstimator(IcpConfig config)
    : config_(std::move(config))
{
}

IcpEstimate PclIcpEstimator::estimate(
    const PointCloud& source_current,
    const PointCloud& target_previous,
    const tf::Transform& T_previous_current_initial) const
{
    (void)source_current;
    (void)target_previous;
    (void)T_previous_current_initial;

    // TODO(ICP): Implement a planar scan-to-scan registration here.
    //
    // Contract:
    //   - source_current is the current scan in the current scan-start laser frame.
    //   - target_previous is the previous scan in the previous scan-start laser frame.
    //   - T_previous_current_initial maps current -> previous and comes from odom.
    //   - a successful result must map current -> previous too.
    //   - return only SE(2): x, y and yaw.  Reject z, roll or pitch.
    //
    // IcpOdometry already performs voxel filtering, first-frame caching, odom-prior
    // construction, quality gates, and RViz diagnostics.  Keep those concerns out
    // of this method so this is the only ICP solver implementation point.
    IcpEstimate result;
    result.status = IcpEstimateStatus::kNotImplemented;
    result.message = "ICP solver TODO: implement PclIcpEstimator::estimate";
    return result;
}

IcpOdometry::IcpOdometry(std::unique_ptr<IcpEstimator> estimator, IcpConfig config)
    : estimator_(std::move(estimator)), config_(std::move(config))
{
}

IcpProcessResult IcpOdometry::process(const PointCloud& current_local_cloud,
                                      const tf::Transform& T_odom_laser_current,
                                      const ros::Time& current_stamp)
{
    IcpProcessResult result;
    result.current_stamp = current_stamp;

    if (!estimator_)
    {
        result.status = IcpProcessStatus::kEstimatorFailed;
        result.message = "No ICP estimator was configured";
        return result;
    }

    if (!isLevelSensorPose(T_odom_laser_current))
    {
        result.status = IcpProcessStatus::kEstimatorFailed;
        result.message = "Current odom-to-laser pose is not a finite level sensor pose";
        return result;
    }

    PointCloudPtr current_filtered = voxelDownsample(current_local_cloud);
    if (!current_filtered || current_filtered->points.size() < config_.minimum_points)
    {
        result.status = IcpProcessStatus::kInsufficientPoints;
        result.message = "Current scan has too few valid points after filtering";
        return result;
    }

    if (!previous_scan_)
    {
        cacheCurrent(std::move(current_filtered), T_odom_laser_current, current_stamp);
        result.status = IcpProcessStatus::kFirstScanCached;
        result.message = "First scan cached; ICP starts from the next usable scan";
        return result;
    }

    if (previous_scan_->cloud->points.size() < config_.minimum_points)
    {
        result.status = IcpProcessStatus::kInsufficientPoints;
        result.message = "Previous scan has too few valid points after filtering";
        return result;
    }

    // T_previous_current = inverse(T_odom_previous) * T_odom_current.
    // This maps current local points into the previous local scan frame.
    const tf::Transform T_previous_current_prior =
        previous_scan_->T_odom_laser.inverse() * T_odom_laser_current;

    result.T_previous_current_prior = T_previous_current_prior;
    result.previous_stamp = previous_scan_->stamp;
    result.previous_cloud_in_odom = transformCloud(
        *previous_scan_->cloud, previous_scan_->T_odom_laser);

    // The transform below intentionally uses the previous pose.  The ICP output
    // lands in the previous scan frame, not the current scan frame.
    result.current_before_in_odom = transformCloud(
        *current_filtered, previous_scan_->T_odom_laser * T_previous_current_prior);

    const IcpEstimate estimate = estimator_->estimate(
        *current_filtered, *previous_scan_->cloud, T_previous_current_prior);
    result.fitness = estimate.fitness;
    result.correspondence_count = estimate.correspondence_count;
    result.T_previous_current = estimate.T_target_source;
    result.message = estimate.message;

    switch (estimate.status)
    {
    case IcpEstimateStatus::kNotImplemented:
        result.status = IcpProcessStatus::kEstimatorNotImplemented;
        return result;
    case IcpEstimateStatus::kNotConverged:
        result.status = IcpProcessStatus::kEstimatorNotConverged;
        return result;
    case IcpEstimateStatus::kInvalidInput:
    case IcpEstimateStatus::kFailed:
        result.status = IcpProcessStatus::kEstimatorFailed;
        return result;
    case IcpEstimateStatus::kSuccess:
        break;
    }

    std::string rejection_reason;
    if (!accepts(estimate, T_previous_current_prior, &rejection_reason))
    {
        result.status = IcpProcessStatus::kRejected;
        result.message = rejection_reason;
        return result;
    }

    result.status = IcpProcessStatus::kAccepted;
    result.accepted = true;
    result.current_aligned_in_odom = transformCloud(
        *current_filtered, previous_scan_->T_odom_laser * estimate.T_target_source);

    // Only an accepted scan advances the *registration target*. A bad
    // registration must not become that target and propagate an incorrect match.
    cacheCurrent(std::move(current_filtered), T_odom_laser_current, current_stamp);
    return result;
}

PointCloudPtr IcpOdometry::voxelDownsample(const PointCloud& input) const
{
    // PCL filters and a future ICP solver must only see finite points.  This also
    // guarantees that the cached target cloud is valid when filtering is disabled.
    PointCloudPtr finite_input = compactFiniteCloud(input);

    if (config_.voxel_leaf_size <= 0.0)
    {
        return finite_input;
    }

    PointCloudPtr output(new PointCloud{});
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(finite_input);
    const float leaf_size = static_cast<float>(config_.voxel_leaf_size);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*output);

    // VoxelGrid should preserve finiteness, but retain this contract explicitly.
    return compactFiniteCloud(*output);
}

bool IcpOdometry::accepts(const IcpEstimate& estimate,
                          const tf::Transform& T_previous_current_prior,
                          std::string* rejection_reason) const
{
    if (!estimate.converged)
    {
        *rejection_reason = "ICP reported success but did not converge";
        return false;
    }

    if (!isFinite(estimate.fitness) || estimate.fitness > config_.maximum_fitness)
    {
        *rejection_reason = "ICP fitness is non-finite or above the configured threshold";
        return false;
    }

    if (estimate.correspondence_count < config_.minimum_correspondences)
    {
        *rejection_reason = "ICP has too few correspondences";
        return false;
    }

    if (!isPlanarTransform(estimate.T_target_source))
    {
        *rejection_reason = "ICP estimate is not a finite SE(2) transform";
        return false;
    }

    if (!isPlanarTransform(T_previous_current_prior))
    {
        *rejection_reason = "Odometry prior is not a finite SE(2) transform";
        return false;
    }

    // Both transforms map current -> previous.  Their relative transform measures
    // the correction ICP applies beyond the odometry prediction.
    const tf::Transform correction =
        estimate.T_target_source * T_previous_current_prior.inverse();
    const double correction_translation = correction.getOrigin().length();
    const double correction_yaw = std::abs(tf::getYaw(correction.getRotation()));

    if (correction_translation > config_.maximum_translation_correction)
    {
        *rejection_reason = "ICP translation correction is too far from the odom prior";
        return false;
    }

    if (correction_yaw > config_.maximum_yaw_correction)
    {
        *rejection_reason = "ICP yaw correction is too far from the odom prior";
        return false;
    }

    return true;
}

void IcpOdometry::cacheCurrent(PointCloudPtr cloud,
                               const tf::Transform& T_odom_laser,
                               const ros::Time& stamp)
{
    std::unique_ptr<PreviousScan> next_scan(new PreviousScan{});
    next_scan->cloud = std::move(cloud);
    next_scan->T_odom_laser = T_odom_laser;
    next_scan->stamp = stamp;
    previous_scan_ = std::move(next_scan);
}

const char* toString(const IcpProcessStatus status) noexcept
{
    switch (status)
    {
    case IcpProcessStatus::kFirstScanCached:
        return "first_scan_cached";
    case IcpProcessStatus::kInsufficientPoints:
        return "insufficient_points";
    case IcpProcessStatus::kEstimatorNotImplemented:
        return "estimator_not_implemented";
    case IcpProcessStatus::kEstimatorNotConverged:
        return "estimator_not_converged";
    case IcpProcessStatus::kEstimatorFailed:
        return "estimator_failed";
    case IcpProcessStatus::kRejected:
        return "rejected";
    case IcpProcessStatus::kAccepted:
        return "accepted";
    }

    return "unknown";
}

}  // namespace laser_undistortion
