# champion_nav_msgs（ROS Noetic）

本目录是 LaserMotion 作业使用的自定义消息工作空间，提供
`champion_nav_msgs/ChampionNavLaserScan`。不要将其安装到 `/opt/ros`；
请在本地工作空间中编译，并在编译/运行主工程前 source 它的 overlay。

```bash
source /opt/ros/noetic/setup.bash

cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

然后在**同一个 shell** 中编译并运行去畸变工作空间：

```bash
cd ../LaserUndistortion_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

若使用本仓库的 Docker 环境，宿主仓库在容器中挂载为 `/workspace`，相应路径为：

```bash
cd /workspace/src/LaserMotion/champion_nav_msgs
```
