#include "../include/calib_odom/Odom_Calib.hpp"

#include <stdexcept>

//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    if(len <= 0)
    {
        throw std::invalid_argument("The calibration data length must be positive.");
    }

    data_len = len;
    now_len = 0;
    A.resize(data_len * 3, 9);
    b.resize(data_len * 3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

构建最小二乘需要的超定方程组 Ax = b。

设校正矩阵按行展开为：
    x = [m00, m01, m02, m10, ..., m22]^T
且 scan = M * odom。每一个观测样本提供三个标量方程。
*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{
    if(data_len <= 0 || now_len >= data_len)
    {
        return false;
    }

    const int row = now_len * 3;

    // Odom 是 3×1 列向量 [ox, oy, oθ]ᵀ；A.block<1, 3>() 是 1×3 行块，
    // 因此 transpose() 仅将其排版成 [ox, oy, oθ]，以匹配赋值目标的尺寸。
    // 它不改变 odom 的数值、物理含义或所属坐标系。
    A.block<1, 3>(row, 0) = Odom.transpose();
    A.block<1, 3>(row + 1, 3) = Odom.transpose();
    A.block<1, 3>(row + 2, 6) = Odom.transpose();

    // b 是一维动态列向量；segment<3>(row) 取从 b[row] 起连续的 3 个元素，
    // 即 b[row]、b[row+1]、b[row+2]。将 3×1 的 scan=[sx, sy, sθ]ᵀ 写入其中，
    // 它们就是当前样本在 Ax=b 右侧对应的三个观测值。
    b.segment<3>(row) = scan;

    ++now_len;
    return true;
}

/*
 * 求解线性最小二乘 Ax = b，并将按行排列的 9 个未知数还原为 3 x 3 矫正矩阵。
 */
Eigen::Matrix3d OdomCalib::Solve()
{
    if(now_len == 0)
    {
        throw std::runtime_error("Cannot solve odometry calibration without samples.");
    }

    const Eigen::Index rows = static_cast<Eigen::Index>(now_len) * 3;
    const Eigen::MatrixXd observations = A.topRows(rows);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(observations);
    if(decomposition.rank() < 9)
    {
        throw std::runtime_error(
            "Cannot solve odometry calibration: the collected motions do not identify all 9 matrix parameters.");
    }

    const Eigen::VectorXd solution = decomposition.solve(b.head(rows));
    if(!solution.allFinite())
    {
        throw std::runtime_error("Cannot solve odometry calibration: least-squares result is not finite.");
    }

    Eigen::Matrix3d correct_matrix;
    correct_matrix.row(0) = solution.segment<3>(0).transpose();
    correct_matrix.row(1) = solution.segment<3>(3).transpose();
    correct_matrix.row(2) = solution.segment<3>(6).transpose();
    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full() const
{
    return data_len > 0 && now_len >= data_len;
}

bool OdomCalib::has_data() const
{
    return now_len > 0;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
    now_len = 0;
}
