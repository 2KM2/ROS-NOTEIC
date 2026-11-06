#ifndef LASER_UNDISTORTION_ICP_ODOMETRY_HPP_
#define LASER_UNDISTORTION_ICP_ODOMETRY_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/time.h>
#include <tf/transform_datatypes.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>

namespace laser_undistortion
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

/**
 * @brief Parameters for the non-algorithm parts of scan-to-scan ICP.
 *
 * All distances are metres and all angles are radians.  The values are
 * deliberately ROS parameters in the node, so tuning does not require a
 * source-code edit.
 */
struct IcpConfig
{
    double voxel_leaf_size{0.05};
    std::size_t minimum_points{30};
    // PCL's basic ICP API does not expose a portable correspondence count;
    // zero disables this optional gate until a solver reports it.
    std::size_t minimum_correspondences{0};
    double maximum_fitness{0.10};
    double maximum_translation_correction{0.50};
    double maximum_yaw_correction{0.5235987755982988};  // 30 degrees.
};

/**
 * @brief Result states distinguish unfinished work from a bad registration.
 */
enum class IcpEstimateStatus
{
    kSuccess,
    kNotImplemented,
    kNotConverged,
    kInvalidInput,
    kFailed
};

/**
 * @brief Output of the ICP solver only.
 *
 * The transform follows the convention T_target_source: it transforms a point
 * expressed in the source (current scan) frame into the target (previous scan)
 * frame.
 */
struct IcpEstimate
{
    IcpEstimateStatus status{IcpEstimateStatus::kNotImplemented};
    bool converged{false};
    std::size_t correspondence_count{0};
    double fitness{std::numeric_limits<double>::infinity()};
    tf::Transform T_target_source{tf::Transform::getIdentity()};
    std::string message;
};

/**
 * @brief Interface that isolates the algorithm students will implement.
 *
 * source_current is always the current scan and target_previous is always the
 * previous scan.  The initial guess and returned transform are both
 * T_previous_current.  This naming is intentional: it makes an accidentally
 * reversed ICP source/target order visible at the call site.
 */
class IcpEstimator
{
public:
    virtual ~IcpEstimator() = default;

    virtual IcpEstimate estimate(
        const PointCloud& source_current,
        const PointCloud& target_previous,
        const tf::Transform& T_previous_current_initial) const = 0;
};

/**
 * @brief Placeholder for the PCL ICP algorithm implementation.
 *
 * Its method is intentionally the only ICP-solver TODO.  The surrounding
 * IcpOdometry class already owns caching, odometry priors, filters, quality
 * gates, and diagnostic clouds.
 */
class PclIcpEstimator final : public IcpEstimator
{
public:
    explicit PclIcpEstimator(IcpConfig config);

    IcpEstimate estimate(
        const PointCloud& source_current,
        const PointCloud& target_previous,
        const tf::Transform& T_previous_current_initial) const override;

private:
    IcpConfig config_;
};

enum class IcpProcessStatus
{
    kFirstScanCached,
    kInsufficientPoints,
    kEstimatorNotImplemented,
    kEstimatorNotConverged,
    kEstimatorFailed,
    kRejected,
    kAccepted
};

/**
 * @brief Registration output plus clouds prepared for RViz diagnostics.
 *
 * All non-null diagnostic clouds are already expressed in odom.  An aligned
 * cloud is present only after an estimate passes every quality gate.
 */
struct IcpProcessResult
{
    IcpProcessStatus status{IcpProcessStatus::kFirstScanCached};
    bool accepted{false};
    double fitness{std::numeric_limits<double>::infinity()};
    std::size_t correspondence_count{0};

    // Odom's prediction and ICP's estimate both map current -> previous.
    tf::Transform T_previous_current_prior{tf::Transform::getIdentity()};
    tf::Transform T_previous_current{tf::Transform::getIdentity()};

    PointCloudPtr previous_cloud_in_odom;
    PointCloudPtr current_before_in_odom;
    PointCloudPtr current_aligned_in_odom;
    ros::Time previous_stamp;
    ros::Time current_stamp;
    std::string message;
};

/**
 * @brief Scan-to-scan ICP infrastructure, excluding the ICP solver itself.
 *
 * Input clouds must already be motion compensated and be expressed in the
 * local laser frame at their own scan-start timestamp.  The class retains only
 * the immediately preceding filtered scan; it neither builds a map nor
 * publishes TF.
 */
class IcpOdometry
{
public:
    explicit IcpOdometry(std::unique_ptr<IcpEstimator> estimator,
                         IcpConfig config);

    IcpProcessResult process(const PointCloud& current_local_cloud,
                             const tf::Transform& T_odom_laser_current,
                             const ros::Time& current_stamp);

private:
    struct PreviousScan
    {
        PointCloudPtr cloud;
        tf::Transform T_odom_laser{tf::Transform::getIdentity()};
        ros::Time stamp;
    };

    PointCloudPtr voxelDownsample(const PointCloud& input) const;
    bool accepts(const IcpEstimate& estimate,
                 const tf::Transform& T_previous_current_prior,
                 std::string* rejection_reason) const;
    void cacheCurrent(PointCloudPtr cloud,
                      const tf::Transform& T_odom_laser,
                      const ros::Time& stamp);

    std::unique_ptr<IcpEstimator> estimator_;
    IcpConfig config_;
    std::unique_ptr<PreviousScan> previous_scan_;
};

const char* toString(IcpProcessStatus status) noexcept;

}  // namespace laser_undistortion

#endif  // LASER_UNDISTORTION_ICP_ODOMETRY_HPP_
