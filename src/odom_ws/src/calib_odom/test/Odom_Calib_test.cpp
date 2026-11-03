#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

#include "calib_odom/Odom_Calib.hpp"

namespace
{

TEST(OdomCalibTest, RecoversKnownLinearCorrectionMatrix)
{
    OdomCalib calibration;
    calibration.Set_data_len(5);

    Eigen::Matrix3d expected;
    expected << 1.02, 0.03, -0.01,
                -0.02, 0.98, 0.04,
                 0.01, -0.03, 1.01;

    const std::array<Eigen::Vector3d, 5> odom_samples = {
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(1.0, -2.0, 0.5),
        Eigen::Vector3d(-0.5, 0.25, -1.0)};

    for(const Eigen::Vector3d& odom : odom_samples)
    {
        EXPECT_TRUE(calibration.Add_Data(odom, expected * odom));
    }

    EXPECT_TRUE(calibration.is_full());
    EXPECT_TRUE(calibration.Solve().isApprox(expected, 1e-12));
}

TEST(OdomCalibTest, RefusesSamplesBeyondConfiguredCapacity)
{
    OdomCalib calibration;
    calibration.Set_data_len(1);

    EXPECT_TRUE(calibration.Add_Data(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX()));
    EXPECT_TRUE(calibration.is_full());
    EXPECT_FALSE(calibration.Add_Data(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY()));
}

TEST(OdomCalibTest, RejectsRankDeficientObservations)
{
    OdomCalib calibration;
    calibration.Set_data_len(3);

    for(int sample = 0; sample < 3; ++sample)
    {
        ASSERT_TRUE(calibration.Add_Data(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX()));
    }

    EXPECT_THROW(calibration.Solve(), std::runtime_error);
}

TEST(OdomCalibTest, ClearingDataAlsoClearsSampleCount)
{
    OdomCalib calibration;
    calibration.Set_data_len(3);

    const Eigen::Matrix3d discarded_matrix = 2.0 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d expected_matrix =
        (Eigen::Matrix3d() << 1.0, 0.1, 0.0,
                              0.0, 0.9, 0.2,
                              -0.1, 0.0, 1.1)
            .finished();

    for(int axis = 0; axis < 3; ++axis)
    {
        const Eigen::Vector3d odom = Eigen::Vector3d::Unit(axis);
        ASSERT_TRUE(calibration.Add_Data(odom, discarded_matrix * odom));
    }
    ASSERT_TRUE(calibration.is_full());

    calibration.set_data_zero();
    EXPECT_FALSE(calibration.is_full());

    for(int axis = 0; axis < 3; ++axis)
    {
        const Eigen::Vector3d odom = Eigen::Vector3d::Unit(axis);
        ASSERT_TRUE(calibration.Add_Data(odom, expected_matrix * odom));
    }

    EXPECT_TRUE(calibration.Solve().isApprox(expected_matrix, 1e-12));
}

}  // namespace
