# LaserMotion：RViz 可视化使用说明

本说明对应 ROS Noetic 版 `LaserMotion` 作业工程。

工程现在会将去畸变前、后的激光点云发布为 RViz 可直接显示的标准 `sensor_msgs/PointCloud2` 消息；启动 `LaserUndistortion.launch` 时也会自动加载预设 RViz 配置。

> 作业核心函数 `Lidar_MotionCalibration()` 仍保留为 TODO。因此在完成 TODO 前，红色和绿色点云会基本重合；这表示 RViz 管线正常，不代表已完成去畸变。

---

## 1. 相关文件

| 文件 | 作用 |
| --- | --- |
| `../LaserUndistortion_ws/src/LaserUndistortion/launch/LaserUndistortion.launch` | 启动去畸变节点和 RViz |
| `../LaserUndistortion_ws/src/LaserUndistortion/launch/LaserMotion.rviz` | RViz 固定坐标系、显示项、颜色、视角配置 |
| `../LaserUndistortion_ws/src/LaserUndistortion/src/LidarMotionUndistortion.cpp` | 发布原始、校正后点云的话题 |
| `../bag/laser.bag` | 回放数据包 |

---

## 2. RViz 显示的内容

启动后，RViz 的 **Fixed Frame** 已设为 `odom`，不需要手动修改。

| RViz 显示项 | 话题 / 坐标系 | 颜色 | 含义 |
| --- | --- | --- | --- |
| Grid | `odom` | 灰色 | 地面坐标网格 |
| TF | `/tf` | 多色坐标轴 | `odom`、`base_link`、`base_laser` 等坐标系关系 |
| Odometry | `/odom` | 黄色 | bag 中的里程计姿态箭头 |
| Raw Cloud | `/laser_motion/raw_cloud` | 红色 | 未做运动去畸变的原始激光点云 |
| Corrected Cloud | `/laser_motion/corrected_cloud` | 绿色 | 调用 `Lidar_MotionCalibration()` 后得到的校正点云 |

红、绿点云均带 `odom` 坐标系和扫描起始时刻的时间戳。节点会把极坐标激光点转换为平面点云，并用该扫描起始时刻的 `base_laser` 位姿放到 `odom` 坐标系中。

---

## 3. 首次构建

两个工程是相互独立的 Catkin 工作空间，必须先构建消息包、source 其 overlay，再构建主工程。

```bash
source /opt/ros/noetic/setup.bash

# 1. 构建自定义激光消息
cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# 2. 构建去畸变节点
cd ../LaserUndistortion_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

构建成功后应生成：

```text
LaserUndistortion_ws/devel/lib/LaserUndistortion/LaserUndistortion_node
```

如果使用本仓库 Docker 环境，则仓库根在容器内的路径为 `/workspace`，上述路径应替换为：

```bash
/workspace/src/LaserMotion/champion_nav_msgs
/workspace/src/LaserMotion/LaserUndistortion_ws
```

---

## 4. 通过 Docker 启动

仓库根目录会以 `/workspace` 挂载到容器中。请始终通过仓库提供的 `docker/run.sh` 启动：它会自动准备 `DISPLAY`、X11/Xwayland 授权和 ROS 环境。**不要直接使用 `docker exec`**，否则可能因缺少或过期的图形授权而无法启动 RViz。

### 4.1 首次构建

在宿主机终端执行。若镜像尚未构建，先运行 `./build_image.sh`：

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker

# 构建 Docker 镜像（首次需要）
./build_image.sh

# 1. 构建自定义激光消息包
./run.sh "source /opt/ros/noetic/setup.bash && \
  cd /workspace/src/LaserMotion/champion_nav_msgs && \
  catkin_make -DCMAKE_BUILD_TYPE=Release"

# 2. 构建去畸变节点（依赖上一步生成的消息）
./run.sh "source /opt/ros/noetic/setup.bash && \
  source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash && \
  cd /workspace/src/LaserMotion/LaserUndistortion_ws && \
  catkin_make -DCMAKE_BUILD_TYPE=Release"
```

镜像或 Dockerfile 更新后，需要用下面的命令销毁并重建常驻容器，再重新执行构建命令：

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker
./run.sh --recreate
```

### 4.2 终端 A：启动节点和 RViz

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker

./run.sh "source /opt/ros/noetic/setup.bash && \
  source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash && \
  source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash && \
  roslaunch LaserUndistortion LaserUndistortion.launch"
```

若宿主机已经有一个 ROS master，此命令会连接到它；否则 `roslaunch` 会自动启动 `roscore`。

### 4.3 终端 B：播放 bag

另开一个宿主机终端：

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker

./run.sh "source /opt/ros/noetic/setup.bash && \
  source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash && \
  source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash && \
  cd /workspace/src/LaserMotion/bag && \
  rosbag play --clock laser.bag"
```

如需循环播放：

```bash
./run.sh "source /opt/ros/noetic/setup.bash && \
  source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash && \
  source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash && \
  cd /workspace/src/LaserMotion/bag && \
  rosbag play --clock --loop laser.bag"
```

`--clock` 不能省略：该 launch 默认设置 `use_sim_time=true`，没有 `/clock` 时 ROS 时间会停在 `0`，RViz 与 TF 的时间同步可能失败。

### 4.4 Docker 图形转发故障

请从宿主机的图形桌面会话终端运行上述命令。若 RViz 报 `Authorization required` 或无法连接显示器，可在该宿主机终端临时执行：

```bash
xhost +local:
```

然后重新执行“终端 A”的启动命令。该设置会放宽本机 X11 访问控制；排障完成后可用 `xhost -local:` 撤销。

---

## 5. 非 Docker：用 `laser.bag` 启动 RViz

### 终端 A：启动节点和 RViz

先确保没有遗留的同名节点、bag 回放进程或第二个 ROS master。

```bash
source /opt/ros/noetic/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

roslaunch LaserUndistortion LaserUndistortion.launch
```

该 launch 会：

1. 设置 `use_sim_time=true`；
2. 启动 `/LaserUndistortion_Node`；
3. 启动 RViz；
4. 自动加载 `LaserMotion.rviz`。

### 终端 B：回放 bag

```bash
source /opt/ros/noetic/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/bag
rosbag play --clock laser.bag
```

必须使用 ASCII 双短横线的 `--clock`。launch 默认启用了仿真时间；若不发布 `/clock`，ROS 时间会停在 0，TF 的按时间查询和 RViz 的时间同步可能失败。

想连续反复观察，可用：

```bash
rosbag play --clock --loop laser.bag
```

---

## 6. 如何判断结果

### TODO 尚未完成时

`Lidar_MotionCalibration()` 还没有修改 `ranges` 和 `angles`，所以：

- 红色原始点云和绿色校正点云应基本重合；
- 两个 RViz 话题都能显示，说明消息发布、TF、RViz 配置的通路正常；
- **不应**据此认为去畸变已完成。

### TODO 完成后

机器人运动时，不同激光束的采样位姿不同。正确补偿后通常应看到：

- 绿色点云的墙面、边缘和直线结构更集中；
- 相比红色点云，绿色点云的重影、拖影或弯曲减小；
- 机器人不动时，两色点云依然应重合。

可以在 RViz 左侧 Displays 面板临时取消勾选任一颜色，单独观察原始或校正结果。

---

## 7. 手动启动 RViz（可选）

如果只想打开配置，不通过 launch 自动启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

rviz -d "$(rospack find LaserUndistortion)/launch/LaserMotion.rviz"
```

但若没有另行启动 `LaserUndistortion_node` 和播放 bag，RViz 只会看到空显示项或状态提示，不会出现点云。

---

## 8. 实时传感器模式（不播放 bag）

默认 launch 用于 `laser.bag` 回放，因此 `use_sim_time` 默认为 `true`。若改为接收实时 `/champion_scan`、`/tf`、`/odom`，且系统没有 `/clock` 发布者，应关闭仿真时间：

```bash
roslaunch LaserUndistortion LaserUndistortion.launch use_sim_time:=false
```

实时模式必须确保：

- 自定义消息话题为 `/champion_scan`；
- TF 能提供同一时间点的 `/odom -> /base_laser`；
- 时间戳使用当前 ROS 时间，而不是历史 bag 时间。

---

## 9. 验证话题与坐标系

播放 bag 后，可在新终端执行：

```bash
rostopic list | grep '^/laser_motion/'
rostopic type /laser_motion/raw_cloud
rostopic type /laser_motion/corrected_cloud
rosnode list
rosrun tf tf_echo /odom /base_laser
```

预期结果：

```text
/laser_motion/raw_cloud
/laser_motion/corrected_cloud

sensor_msgs/PointCloud2
sensor_msgs/PointCloud2
```

也可检查点云头部：

```bash
rostopic echo -n 1 /laser_motion/raw_cloud/header
rostopic echo -n 1 /laser_motion/corrected_cloud/header
```

两个话题的 `frame_id` 应为：

```text
odom
```

---

## 10. 常见问题

### 10.1 RViz 没有自动弹出

检查 launch 是否能找到 RViz：

```bash
rosrun rviz rviz --help
```

在 Docker 中运行时，还需确认宿主机的 X11/Wayland 图形转发可用。请使用仓库的 `docker/run.sh` 进入容器，而不是自行执行一个没有 `DISPLAY` / Xauthority 挂载的 `docker exec`。

### 10.2 RViz 中出现 `No transform` 或 Fixed Frame 错误

依次检查：

1. RViz 的 Fixed Frame 是否仍为 `odom`；
2. bag 是否使用 `rosbag play --clock laser.bag`；
3. `/tf` 是否正在发布；
4. `rosrun tf tf_echo /odom /base_laser` 是否能输出变换；
5. 是否误启动了多个 `roscore` 或连接到错误的 `ROS_MASTER_URI`。

### 10.3 有 TF、里程计，但没有红绿点云

检查节点和话题：

```bash
rosnode info /LaserUndistortion_Node
rostopic hz /champion_scan
rostopic hz /laser_motion/raw_cloud
rostopic hz /laser_motion/corrected_cloud
```

如果终端出现 `Not Start Pose`、`No Transform` 或 `Extrapolation`，说明在当前扫描时间戳无法查询 `/odom -> /base_laser`，优先按 10.2 排查。

### 10.4 红色与绿色完全重合

若 TODO 尚未完成，这是预期行为。完成 `Lidar_MotionCalibration()` 后，确保函数确实原地更新了传入的 `ranges` 和 `angles`，再观察绿色结构是否更集中。

---

## 11. 关键结论

用于本作业回放的最短流程是：

```bash
# 终端 A
roslaunch LaserUndistortion LaserUndistortion.launch

# 终端 B
cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/bag
rosbag play --clock laser.bag
```

启动完成后，RViz 会自动显示 `odom`、TF、黄色里程计、红色原始点云和绿色校正点云。完成作业 TODO 后，使用红绿点云的差异即可直观验证运动去畸变效果。
