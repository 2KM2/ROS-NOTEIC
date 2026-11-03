#include <exception>
#include <memory>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "boost/asio.hpp"   //包含boost库函数
#include "boost/bind.hpp"
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "../include/calib_odom/Odom_Calib.hpp"

#include <csm/csm_all.h>

using namespace std;
using namespace boost::asio;//定义一个命名空间，用于后面的读写操作
//using namespace karto;

//用来进行里程计矫正的类
OdomCalib Odom_calib;


std::vector<geometry_msgs::PointStamped> mcu_path;

Eigen::Vector3d  cal_delta_distance(Eigen::Vector3d odom_pose);
/*
 * 获取激光数据类
*/
class Scan2
{
public:
    Scan2();
    ~Scan2();

    //进行PI-ICP需要的变量
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    //odom & scan　进行位姿积分
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;
    bool has_previous_odom_pose_;

    std::vector<Eigen::Vector3d> odom_increments;          //用来储存两帧之间的里程计的增量

    std::string odom_frame_;
    std::string base_frame_;

    ros::NodeHandle node_;
    tf::TransformListener tf_;

    ros::Subscriber calib_flag_sub_;

    ros::Publisher odom_path_pub_,scan_path_pub_,calib_path_pub_;

    nav_msgs::Path path_odom,path_scan;
    ros::Time current_time;

    //进行时间同步
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > scan_filter_sub_;
    std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan> > scan_filter_;

    void CalibFlagCallBack(const std_msgs::Empty &msg);

    //回调函数
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr&scan2);

    //tf树查询里程计位姿
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);

    //发布odom & laser path
    void pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_);

    //为了发布correct path
    void publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_);

    //进行pl-icp的相关函数.
    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                                 LDP& ldp);
    bool PIICPBetweenTwoFrames(LDP& currentLDPScan,
                               const Eigen::Vector3d& initial_pose,
                               Eigen::Vector3d& relative_pose);


};


void Scan2:: pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_)
{

    current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);
    mcu_path_pub_.publish(path);

}

void Scan2::publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_)
{
    nav_msgs::Path visual_path;

    current_time = ros::Time::now();

    visual_path.header.stamp = ros::Time::now();
    visual_path.header.frame_id="odom";

    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.stamp = current_time;
    tmpPose.header.frame_id="odom";

    for(int i = 0 ;i < path_eigen.size();i++)
    {
        Eigen::Vector3d poseEigen = path_eigen[i];

        tmpPose.pose.position.x = poseEigen(0);
        tmpPose.pose.position.y = poseEigen(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseEigen(2));
        tmpPose.pose.orientation.x = goal_quat.x;
        tmpPose.pose.orientation.y = goal_quat.y;
        tmpPose.pose.orientation.z = goal_quat.z;
        tmpPose.pose.orientation.w = goal_quat.w;

        visual_path.poses.push_back(tmpPose);
    }

    path_pub_.publish(visual_path);
}

/*
 * 得到时刻t时候 机器人在里程计坐标下的坐标
*/
bool Scan2::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, base_frame_);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame_, ident, odom_pose);
    }


    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());



    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    //pub_msg(pose,path_odom,odom_path_pub_);
    return true;
}


//构造函数
Scan2::Scan2()
{
    ros::NodeHandle private_nh_("~");

    m_prevLDP = NULL;
    SetPIICPParams();

    scan_pos_cal.setZero();
    odom_pos_cal.setZero();
    has_previous_odom_pose_ = false;
    odom_increments.clear();

    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!private_nh_.getParam("base_frame", base_frame_))
        base_frame_ = "base_link";

    //订阅对应的topic 接受到这个topic 系统就开始进行最小二乘的解算
    calib_flag_sub_ = node_.subscribe("calib_flag",5,&Scan2::CalibFlagCallBack,this);

    //发布路径
    odom_path_pub_ = node_.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
    scan_path_pub_ = node_.advertise<nav_msgs::Path>("scan_path_pub_",1,true);
    calib_path_pub_ = node_.advertise<nav_msgs::Path>("calib_path_pub_",1,true);
    current_time = ros::Time::now();

    path_odom.header.stamp=current_time;
    path_scan.header.stamp=current_time;
    path_odom.header.frame_id="odom";
    path_scan.header.frame_id="odom";

    //进行里程计和激光雷达数据的同步
    scan_filter_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "/sick_scan", 10));
    scan_filter_.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 10));
    scan_filter_->registerCallback(boost::bind(&Scan2::scanCallBack, this, _1));

    std::cout <<"Calibration Online,Wait for Data!!!!!!!"<<std::endl;
}

//订阅topic表示开始进行标定.
void Scan2::CalibFlagCallBack(const std_msgs::Empty &msg)
{
    if(!Odom_calib.has_data())
    {
        ROS_WARN("No valid odometry/laser scan pairs have been collected; calibration is skipped.");
        return;
    }

    Eigen::Matrix3d correct_matrix;
    try
    {
        correct_matrix = Odom_calib.Solve();
    }
    catch(const std::exception& error)
    {
        ROS_ERROR("Failed to solve odometry calibration: %s", error.what());
        return;
    }

    Eigen::Matrix3d tmp_transform_matrix;

    std::cout<<"correct_matrix:"<<std::endl<<correct_matrix<<std::endl;

    //计算矫正之后的路径
    Eigen::Vector3d calib_pos(0,0,0);                 //矫正之后的位姿
    std::vector<Eigen::Vector3d> calib_path_eigen;    //矫正之后的路径
    for(int i = 0; i < odom_increments.size();i++)
    {
        Eigen::Vector3d odom_inc = odom_increments[i];
        Eigen::Vector3d correct_inc = correct_matrix * odom_inc;

        tmp_transform_matrix << cos(calib_pos(2)),-sin(calib_pos(2)),0,
                            sin(calib_pos(2)), cos(calib_pos(2)),0,
                                            0,                 0,1;

        calib_pos += tmp_transform_matrix * correct_inc;

        calib_path_eigen.push_back(calib_pos);
    }

    //发布矫正之后的路径
    publishPathEigen(calib_path_eigen,calib_path_pub_);

    //矫正完毕，退出订阅
    scan_filter_sub_->unsubscribe();

    std::cout <<"calibration over!!!!"<<std::endl;
}


//激光数据回调函数
void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg)
{
    static long int dataCnt = 0;
    sensor_msgs::LaserScan scan;
    Eigen::Vector3d odom_pose;              //激光对应的里程计位姿
    Eigen::Vector3d d_point_odom;           //里程计计算的dpose
    Eigen::Vector3d d_point_scan;           //激光的scanmatch计算的dpose
    Eigen::MatrixXd transform_matrix(3,3);  //临时的变量

    double c,s;
    scan = *_laserScanMsg;

    //得到对应的里程计数据
    if(!getOdomPose(odom_pose, _laserScanMsg->header.stamp))
        return ;

    // 先将每一帧激光转换为 CSM 数据。里程计参考帧和激光参考帧必须始终
    // 对应同一时刻：即使运动太小而不建立标定样本，也要推进激光参考帧。
    LDP currentLDP = NULL;
    LaserScanToLDP(&scan, currentLDP);
    if(currentLDP == NULL)
    {
        ROS_WARN("Received an empty laser scan, resetting the scan-matching reference.");
        if(m_prevLDP != NULL)
        {
            ld_free(m_prevLDP);
            m_prevLDP = NULL;
        }
        return;
    }

    // 首帧同时建立里程计和激光参考，不能构成相对运动。
    if(!has_previous_odom_pose_)
    {
        cal_delta_distance(odom_pose);
        has_previous_odom_pose_ = true;
        m_prevLDP = currentLDP;
        return;
    }

    // 前后两帧里程计的位姿差；它与 m_prevLDP/currentLDP 表示同一对激光帧。
    d_point_odom = cal_delta_distance(odom_pose);
    if(m_prevLDP == NULL)
    {
        m_prevLDP = currentLDP;
        return;
    }

    // 如果平移和转角都很小，则不建立标定样本。但仍用当前帧替换参考帧，使
    // 下一次 ICP 使用的两帧与 d_point_odom 使用的两次里程计位姿保持同步。
    // 对局部坐标中的反向运动也必须按绝对值判断，否则后退、右移或顺时针旋转
    // 会错误地被跳过。
    if(std::abs(d_point_odom(0)) < 0.05 &&
       std::abs(d_point_odom(1)) < 0.05 &&
       std::abs(d_point_odom(2)) < tfRadians(5.0))
    {
        ld_free(m_prevLDP);
        m_prevLDP = currentLDP;
        return;
    }

    // d_point_scan 是两帧激光匹配得到的相对位姿。失败的匹配不会被加入
    // 标定方程，但 PIICPBetweenTwoFrames 会保留当前帧作为下一次的参考帧。
    if(!PIICPBetweenTwoFrames(currentLDP, d_point_odom, d_point_scan))
    {
        return;
    }

    // 构造旋转矩阵 生成三种位姿

    // 两针scan计算本身累计的位姿 for laser_path visualization
    c = cos(scan_pos_cal(2));
    s = sin(scan_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;
    scan_pos_cal+=(transform_matrix*d_point_scan);

    // 里程计累计的位姿          for odom_path visualization
    c = cos(odom_pos_cal(2));
    s = sin(odom_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;
    odom_pos_cal+=(transform_matrix*d_point_odom);

    //放到路径当中 //for visualization
    pub_msg(odom_pos_cal,path_odom,odom_path_pub_);
    pub_msg(scan_pos_cal,path_scan,scan_path_pub_);

    // 构造超定方程组。容量达到上限后不再保留额外的路径增量，避免
    // Solve() 使用的样本集合和重放校正路径时使用的增量集合不一致。
    if(!Odom_calib.Add_Data(d_point_odom, d_point_scan))
    {
        ROS_WARN_THROTTLE(5.0, "Calibration sample capacity reached; ignoring further samples.");
        return;
    }

    // 保存与成功写入标定方程的同一组里程计增量。
    odom_increments.push_back(d_point_odom);
    ++dataCnt;

    std::cout <<"Data Cnt:"<<dataCnt<<std::endl;
}

/*
 * 计算当前帧相对于上一帧的里程计增量。
 *
 * 输入的 now_pos 与保存的 last_pos 都表示 base_link 在 odom 全局坐标系下的
 * 绝对位姿 [x, y, yaw]。扫描匹配的输出则表达在上一帧局部坐标系中，因此需要把
 * odom 坐标系中的位置差转换到上一帧车体坐标系，最终返回：
 * [前进量, 左移量, 偏航变化量]。
 */
Eigen::Vector3d cal_delta_distance(Eigen::Vector3d odom_pose)
{
    static Eigen::Vector3d last_pos;
    static bool has_last_pos = false;
    const Eigen::Vector3d now_pos = odom_pose;

    // 第一次调用只建立参考位姿；此时没有前一帧，不能计算运动增量。
    if(!has_last_pos)
    {
        last_pos = now_pos;
        has_last_pos = true;
        return Eigen::Vector3d::Zero();
    }

    // 1. 在 odom 全局坐标系中计算两帧的位置差。
    const double dx_world = now_pos(0) - last_pos(0);
    const double dy_world = now_pos(1) - last_pos(1);

    // 2. 使用上一帧的朝向构造 R(-theta_0)。
    //    R(theta_0) 将局部坐标转到 odom 坐标；其逆 R(-theta_0) 则将
    //    odom 中的位移转换为上一帧 base_link 坐标中的 [前进, 左移] 位移。
    const double theta_0 = last_pos(2);
    const double cos_theta = std::cos(theta_0);
    const double sin_theta = std::sin(theta_0);

    Eigen::Vector3d d_pos;
    d_pos(0) = cos_theta * dx_world + sin_theta * dy_world;
    d_pos(1) = -sin_theta * dx_world + cos_theta * dy_world;

    // 3. 计算偏航角增量，并归一化到 [-pi, pi]。
    //    atan2(sin(delta), cos(delta)) 能处理从 pi 跳到 -pi 的角度回绕。
    const double yaw_delta = now_pos(2) - last_pos(2);
    d_pos(2) = std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));

    // 4. 在所有计算完成后，再将当前绝对位姿保存为下一次调用的参考位姿。
    last_pos = now_pos;

    return d_pos;
}

//设置PI-ICP的参数
void Scan2::SetPIICPParams()
{
    //设置激光的范围
    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 20;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}



//把激光雷达数据 转换为PI-ICP需要的数据
void Scan2::LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                           LDP& ldp)
{
    const int nPts = static_cast<int>(pScan->ranges.size());
    if(nPts == 0)
    {
        ldp = NULL;
        return;
    }

    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > 0.1 && dist < 20)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//求两帧之间的icp位姿匹配
bool Scan2::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                  const Eigen::Vector3d& initial_pose,
                                  Eigen::Vector3d& relative_pose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = initial_pose(0);
    m_PIICPParams.first_guess[1] = initial_pose(1);
    m_PIICPParams.first_guess[2] = initial_pose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    const bool matched = m_OutputResult.valid;
    if(matched)
    {
        //得到两帧激光之间的相对位姿
        relative_pose(0) = m_OutputResult.x[0];
        relative_pose(1) = m_OutputResult.x[1];
        relative_pose(2) = m_OutputResult.x[2];
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
    }

    // 无论本次匹配是否成功，都将当前帧作为下一次的参考帧；但仅将成功
    // 匹配的帧对加入标定样本，避免用里程计初值伪造激光观测。
    ld_free(m_prevLDP);
    m_prevLDP = currentLDPScan;

    return matched;
}

Scan2::~Scan2()
{
    if(m_prevLDP != NULL)
    {
        ld_free(m_prevLDP);
    }
}

int main(int argc,char** argv)
{

    ros::init(argc, argv, "message_filter_node");
    ros::Time::init();
    ros::NodeHandle n;
    Scan2 scan;


    Odom_calib.Set_data_len(12000);
    Odom_calib.set_data_zero();
    ros::spin();

    return 0;
}
