# LaserMotion 的 GDB 启动脚本。
# 从 LaserUndistortion_ws 目录执行：
#   gdb -q -x src/LaserUndistortion/scripts/debug_laser_undistortion.gdb \
#       devel/lib/LaserUndistortion/LaserUndistortion_node
#
# 本脚本不会自动 run：请先在另一个终端使 rosbag 以 --pause 启动，
# 然后在此 GDB 中输入 run。节点会停在第一个激光回调中。
#
# 断点目标：
#   scanCallback()                 一帧数据刚进入节点
#   Lidar_MotionCalibration()      单个 5 ms 分段的去畸变 TODO
#   IcpOdometry::process()         去畸变点云进入 ICP 框架
#   PclIcpEstimator::estimate()    ICP 求解 TODO（当前是占位实现）
set pagination off
set print pretty on
set print elements 20
set breakpoint pending on

break LidarMotionUndistortion.cpp:107
break LidarMotionUndistortion.cpp:270

# icp_odometry.cpp 属于动态加载的共享库。节点启动后 GDB 会自动解析这两个
# pending breakpoints；不需要手动运行 GDB 的 sharedlibrary 命令。
break laser_undistortion::IcpOdometry::process
break laser_undistortion::PclIcpEstimator::estimate

printf "\nGDB breakpoints are ready. Start rosbag in a second terminal, then type: run\n"
printf "Useful commands: bt | frame 1 | info locals | p ranges.size() | p ranges[0] | c\n\n"
