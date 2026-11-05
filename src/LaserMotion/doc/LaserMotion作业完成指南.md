# LaserMotion：基于里程计的激光雷达运动去畸变作业指南

> 本指南对应仓库中的实际文件，而不是旧说明文档中的文件名。数据包是 `../bag/laser.bag`，**不是** `odom.bag`。
>
> 本工程已按 ROS Noetic 迁移并在仓库 Docker 的 Noetic 环境中通过编译。下面第 4 节的 TODO 仍由学生完成；Noetic 迁移不会替你实现这部分作业算法。
>
> 本作业的学生 TODO 位于：
> `../LaserUndistortion_ws/src/LaserUndistortion/src/LidarMotionUndistortion.cpp`
> 的 `LidarMotionCalibrator::Lidar_MotionCalibration()`。

---

## 1. 作业要完成什么

二维激光雷达转一圈需要一段时间。机器人在扫描期间若发生平移或旋转，则不同激光束是在不同的机器人位姿下测得的；如果直接把这些点拼在一起，原本笔直的墙会产生拖影、弯曲或重影，这就是**运动畸变**。

本作业利用 bag 中的 `/tf` / `/odom` 信息，以每帧激光的起始位姿为统一参考，将每束激光点校正到同一时刻（起始时刻）对应的 `base_laser` 坐标系。

工程中的关键对象如下：

| 内容 | 实际位置 / 名称 | 作用 |
| --- | --- | --- |
| 自定义消息包 | `../champion_nav_msgs` | 提供 `champion_nav_msgs/ChampionNavLaserScan` |
| 待完成节点 | `../LaserUndistortion_ws/src/LaserUndistortion/src/LidarMotionUndistortion.cpp` | 订阅激光、查询 TF、校正，并发布 RViz 可显示的点云 |
| 启动文件 | `../LaserUndistortion_ws/src/LaserUndistortion/launch/LaserUndistortion.launch` | 设置仿真时间并启动节点 |
| 回放数据 | `../bag/laser.bag` | 已在 Noetic 中检查：53.0 秒、7,620 条消息，包含 `/champion_scan`、`/tf`、`/odom` |

消息 `ChampionNavLaserScan` 的主要字段为：

```text
Header header              # 第一束激光的时间戳
float32 time_increment     # 相邻两束激光的时间间隔（秒）
float32[] ranges           # 距离
float32[] angles           # 每束激光的角度
```

当前节点订阅 `champion_scan`（无命名空间时即 `/champion_scan`），并查询 `/odom -> /base_laser` 的 TF。

---

## 2. 先理解现有调用流程

```text
ChampionNavLaserScan 回调 ScanCallBack()
  ├─ 提取 ranges / angles
  ├─ 保存原始扫描副本
  ├─ Lidar_Calibration()
  │    ├─ 每 5 ms 取本段起点、终点的 TF 位姿
  │    └─ 调用 Lidar_MotionCalibration()   ← 本作业的核心 TODO
  └─ 将原始 / 校正后点云转换到 odom，并发布给 RViz
```

### 三个位姿分别表示什么

`Lidar_MotionCalibration()` 的参数含义：

| 参数 | 含义 |
| --- | --- |
| `frame_base_pose` | **整帧统一参考位姿**。现有代码将其设为整帧第一束激光的位姿，所有校正后的点必须表达在这个坐标系中。 |
| `frame_start_pose` | 当前 5 ms 分段内第一束激光对应的 `base_laser` 在 `odom` 下的位姿。 |
| `frame_end_pose` | 当前 5 ms 分段内最后一束激光对应的位姿。 |
| `startIndex` | 当前分段第一束激光在 `ranges` / `angles` 中的下标。 |
| `beam_number` | 当前分段的激光束数量。 |

注意：`frame_base_pose` 是**整帧**的基准，不能在每个分段改为 `frame_start_pose`。这样做会让各分段点仍处于不同参考系，无法完成整帧去畸变。

---

## 3. 核心数学：把瞬时点变换到整帧起始坐标系

对当前分段的第 `j` 束激光（`j = 0 ... beam_number - 1`）：

1. 计算其在分段内的时间比例：

   ```text
   alpha = j / (beam_number - 1)
   ```

   当 `beam_number == 1` 时，令 `alpha = 0`，避免除零。

2. 在起、终点位姿之间插值得到该束激光的瞬时位姿 `T_j`。

   - 平移：线性插值；
   - 旋转：四元数球面插值（slerp），避免航向角跨越 `π/-π` 时发生错误的大角度旋转。

3. 将极坐标激光点转换到该时刻激光坐标系：

   ```text
   p_laser = [ range * cos(angle), range * sin(angle), 0 ]
   ```

4. 先转到世界/里程计坐标系，再转回整帧基准坐标系：

   ```text
   p_base = inverse(T_base) * T_j * p_laser
   ```

   其中 `T_base` 即 `frame_base_pose`。这个写法可以避免手推旋转矩阵时把正负号或变换方向写反。

5. 把 `p_base` 转回极坐标，并写回原数组：

   ```text
   ranges[index] = hypot(p_base.x, p_base.y)
   angles[index] = atan2(p_base.y, p_base.x)
   ```

之后节点会将这些已统一到起始激光坐标系的点，用起始位姿转换到 `odom` 下，并发布为绿色 RViz 点云。

---

## 4. 可直接填写的 TODO 实现

在源文件中添加标准头（若编译器尚未通过其他头间接包含它们）：

```cpp
#include <cmath>
#include <vector>
```

然后将 `Lidar_MotionCalibration()` 中的 TODO 替换为以下代码：

```cpp
void Lidar_MotionCalibration(
        tf::Stamped<tf::Pose> frame_base_pose,
        tf::Stamped<tf::Pose> frame_start_pose,
        tf::Stamped<tf::Pose> frame_end_pose,
        std::vector<double>& ranges,
        std::vector<double>& angles,
        int startIndex,
        int& beam_number)
{
    if (beam_number <= 0 || startIndex < 0 ||
        startIndex + beam_number > static_cast<int>(ranges.size()) ||
        ranges.size() != angles.size())
    {
        ROS_WARN("Invalid laser calibration segment");
        return;
    }

    // T_base：整帧起始激光坐标系到 odom 坐标系的变换。
    const tf::Transform base_transform(frame_base_pose.getRotation(),
                                       frame_base_pose.getOrigin());
    const tf::Transform base_inverse = base_transform.inverse();

    const tf::Vector3 start_origin = frame_start_pose.getOrigin();
    const tf::Vector3 end_origin = frame_end_pose.getOrigin();
    const tf::Quaternion start_rotation = frame_start_pose.getRotation();
    const tf::Quaternion end_rotation = frame_end_pose.getRotation();

    for (int j = 0; j < beam_number; ++j)
    {
        const int index = startIndex + j;

        // 保持现有约定：零值和无效点不参与校正。
        if (ranges[index] < 0.05 ||
            std::isnan(ranges[index]) || std::isinf(ranges[index]))
        {
            continue;
        }

        const double alpha = (beam_number == 1)
            ? 0.0
            : static_cast<double>(j) / static_cast<double>(beam_number - 1);

        // 假设当前 5 ms 分段内做匀速平移、匀速转动。
        const tf::Vector3 interpolated_origin =
            start_origin.lerp(end_origin, alpha);
        const tf::Quaternion interpolated_rotation =
            start_rotation.slerp(end_rotation, alpha);
        const tf::Transform beam_transform(interpolated_rotation,
                                           interpolated_origin);

        // 当前瞬时激光坐标系 -> odom -> 整帧起始激光坐标系。
        const double x = ranges[index] * std::cos(angles[index]);
        const double y = ranges[index] * std::sin(angles[index]);
        const tf::Vector3 point_in_laser(x, y, 0.0);
        const tf::Vector3 point_in_base =
            base_inverse * (beam_transform * point_in_laser);

        ranges[index] = std::hypot(point_in_base.x(), point_in_base.y());
        angles[index] = std::atan2(point_in_base.y(), point_in_base.x());
    }
}
```

### 这段代码如何自检

- `j == 0` 时，`alpha == 0`，使用 `frame_start_pose`；
- `j == beam_number - 1` 时，`alpha == 1`，使用 `frame_end_pose`；
- 如果机器人静止，则 `T_j == T_base`，所以每个点变换前后应相同；
- 如果激光点正好在扫描起始时刻采集，`inverse(T_base) * T_base * p == p`，不应产生额外偏移。

---

## 5. 已完成的起止时间与数组顺序修正

这一节不是 TODO 函数本体。为使 Noetic 迁移后的模板能正确对应每束激光的采样时间，已修正原始模板在 `ScanCallBack()` 中的两个边界问题：

1. 原循环从 `beamNum - 1` 遍历到 `1`，导致数组顺序反转且遗漏第 0 束；
2. 一帧有 `N` 束时，最后一束通常在
   `start + (N - 1) * time_increment`，不是 `start + N * time_increment`。

当前 Noetic 版源码已经采用以下逻辑；如果你正在对比旧模板，应确认对应代码一致：

```cpp
const int beamNum = static_cast<int>(laserScanMsg.ranges.size());
if (beamNum == 0 || laserScanMsg.angles.size() != laserScanMsg.ranges.size())
{
    ROS_WARN("Invalid laser scan data");
    return;
}

// 首束位于 startTime；末束位于 startTime + (N - 1) * time_increment。
const ros::Time endTime = startTime + ros::Duration(
    laserScanMsg.time_increment * static_cast<double>(beamNum - 1));

std::vector<double> ranges;
std::vector<double> angles;
ranges.reserve(beamNum);
angles.reserve(beamNum);

for (int i = 0; i < beamNum; ++i)
{
    double lidar_dist = laserScanMsg.ranges[i];
    if (lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
    {
        lidar_dist = 0.0;
    }

    ranges.push_back(lidar_dist);
    angles.push_back(laserScanMsg.angles[i]);
}
```

`Lidar_Calibration()` 中也已将每束间隔改为：

```cpp
const double time_inc = (beamNumber <= 1)
    ? 0.0
    : (end_time - start_time) / static_cast<double>(beamNumber - 1);
```

这样数组下标、采样顺序和激光时间才能一一对应。若老师明确要求保留“倒序扫描”的数据格式，则必须同时倒序**时间索引、距离和角度**，不能只倒序数组。

### 5.1 分段边界不能让同一束被原地校正两次

旧模板在一段结束后执行：

```cpp
start_time = mid_time;
start_index = i;
frame_start_pose = frame_mid_pose;
```

这会使下一个分段从刚处理过的 `i` 号束再次开始。由于 `Lidar_MotionCalibration()` 会**原地**改写 `ranges[index]` 和 `angles[index]`，第二次处理不是得到“相同结果”，而是对已经变换到基准坐标系的点再施加一次变换，产生错误。

当前 Noetic 源码已改为不重叠分段：每段处理闭区间 `[segment_start, segment_end]` 后，下一段从 `segment_end + 1` 开始；同时将段起始时间前进到该新首束的采样时刻，并重新查询该束对应的 TF 位姿。不要只把索引改为 `i + 1`，却继续使用 `i` 号束的时间或 `frame_mid_pose`，否则时间与位姿依然不匹配。

实现时还要保证最后一段包含最后一束，且不要访问数组越界。

> 无论怎样修复分段边界，`frame_base_pose` 仍必须保持整帧第一束的位姿；不能改成每段起点。

---

## 6. ROS Noetic 下的构建准备

### 6.1 `champion_nav_msgs` 是独立工作空间

旧 README 曾使用 `/opt/ros/indigo`；当前 README 与 `install.sh` 已改为 Noetic 本地 overlay 构建，不再把作业消息包安装进 `/opt/ros`。构建命令为：

```bash
source /opt/ros/noetic/setup.bash

cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 6.2 再构建 `LaserUndistortion_ws`

必须在**同一个 shell** 中保留上一步的 overlay，然后构建去畸变工程：

```bash
cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/LaserUndistortion_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

构建成功后应存在：

```text
devel/lib/LaserUndistortion/LaserUndistortion_node
```

### 6.3 若使用本仓库的 Docker Noetic 环境

仓库根目录的 `docker/` 提供 ROS Noetic + GUI 转发环境。首次使用请按 [`../../../docker/README.md`](../../../docker/README.md) 构建镜像；之后进入容器：

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker
./run.sh
```

容器内仓库路径是 `/workspace`，对应的构建命令为：

```bash
source /opt/ros/noetic/setup.bash

cd /workspace/src/LaserMotion/champion_nav_msgs
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

cd /workspace/src/LaserMotion/LaserUndistortion_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

---

## 7. 可能遇到的构建问题（与 TODO 分开处理）

以下列出旧模板中的迁移项。前四项已在当前 Noetic 源码中完成；阅读时可用它们理解迁移原因，遇到基于旧工程的副本也可按此排查。它们不属于去畸变数学。

### 7.1 `champion_nav_msgs/package.xml` 依赖不完整/过时

旧 manifest 曾声明工程中未提供的历史包：

```text
PoseWithPrecision_msg
GraphConstraint_msg
Edge_msg
Node_msg
```

同时 `nav_msgs` 前面曾含有全角空白字符。当前 Noetic `package.xml` 已清理这些无效依赖，仅保留消息定义实际用到的标准依赖：

```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>message_generation</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>nav_msgs</build_depend>

<exec_depend>message_runtime</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
```

### 7.2 `LaserUndistortion` 缺少依赖声明

旧版 `package.xml` 未完整声明源码实际使用的 `tf`、`sensor_msgs`、PCL 与 `champion_nav_msgs`。当前 Noetic 版已：

- 在 `CMakeLists.txt` 的 `find_package(catkin REQUIRED COMPONENTS ...)` 添加 `champion_nav_msgs`；
- 在 `package.xml` 声明 `tf`、`sensor_msgs`、`champion_nav_msgs` 与 `pcl_ros`；
- 增加消息生成目标依赖，保证并行构建时先生成自定义消息；
- 保持先构建、source `champion_nav_msgs` 再构建本工程的顺序。

### 7.3 PCL 版本路径硬编码

旧版源文件曾包含：

```cpp
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
```

Noetic 常见 PCL 版本并非 1.7。当前 Noetic 版已经改用由 CMake 检测到的标准路径：

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
```

### 7.4 程序退出时的非法释放

旧模板中，`main()` 将栈对象 `tf::TransformListener tf` 的地址传入构造函数，但析构函数中又对 `tf_` 执行 `delete`，这是不匹配的所有权关系。当前 Noetic 版析构函数已改为默认实现：

```cpp
~LidarMotionCalibrator() = default;
```

`LidarMotionCalibrator` 不拥有这个指针，因此不应释放它。

---

## 8. 正确的运行顺序

启动前，停止之前残留的 `roscore`、`roslaunch`、`rosbag play` 或同名节点，确保系统中只有一个 ROS master。

### 终端 A：启动去畸变节点

```bash
source /opt/ros/noetic/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

roslaunch LaserUndistortion LaserUndistortion.launch
```

`LaserUndistortion.launch` 提供以下参数，默认适合本作业的 `laser.bag`：

```xml
<arg name="use_sim_time" default="true"/>
<param name="use_sim_time" value="$(arg use_sim_time)"/>
```

该 launch 也会自动以 `LaserMotion.rviz` 配置启动 RViz，显示：

- 红色 `/laser_motion/raw_cloud`：去畸变前的点云；
- 绿色 `/laser_motion/corrected_cloud`：去畸变后的点云（完成 TODO 前会与红色重合）；
- 黄色 `/odom` 里程计箭头与 TF 坐标系。

两个点云都已转换为 `odom` 坐标系并带有 bag 时间戳，因此 RViz 可以在去畸变算法完成后直接对比红绿结果。不要手动改 Fixed Frame；保持 `odom`。

使用实时传感器而非 bag 时，在启动 launch 时关闭仿真时间：

```bash
roslaunch LaserUndistortion LaserUndistortion.launch use_sim_time:=false
```

在没有 master 时，`roslaunch` 会启动一个；因此不要再手动启动第二个 `roscore`。

### 终端 B：回放数据包

```bash
source /opt/ros/noetic/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/bag
rosbag play --clock laser.bag
```

关键点：

- 参数必须是 ASCII 的双短横线 `--clock`，不要从 Word 中复制排版短横线 `–clock`；
- 因为启用了 `use_sim_time`，不加 `--clock` 会使节点难以得到与 bag 对应的仿真时间，TF 查询可能失败；
- 需要反复检查时可使用：

  ```bash
  rosbag play --clock --loop laser.bag
  ```

---

## 9. 验收与排错清单

### 9.1 ROS 与 TF 是否正常

bag 开始播放后，在新终端（同样 source 两个工作空间）检查：

```bash
rostopic type /champion_scan
rosnode list
rosnode info /LaserUndistortion_Node
rosrun tf tf_echo /odom /base_laser
```

预期：

- `/champion_scan` 的类型是 `champion_nav_msgs/ChampionNavLaserScan`；
- 节点列表中存在 `/LaserUndistortion_Node`；
- `tf_echo` 在 bag 播放期间持续输出变换；
- 终端中不应持续出现 `Not visualPose`、`No Transform`、`Extrapolation` 等错误。

若 TF 报错，优先检查：是否先启动 launch、是否带 `--clock` 回放、是否有多个 ROS master，以及 bag 是否已播放结束。

### 9.2 RViz 可视化结果是否符合预期

启动 launch 后会自动打开 RViz，直接比较：

- **红色点云**（`/laser_motion/raw_cloud`）：未校正的原始扫描；
- **绿色点云**（`/laser_motion/corrected_cloud`）：校正后的扫描；
- **黄色箭头**：`/odom` 里程计；
- **TF**：`odom`、`base_link` 与 `base_laser` 等坐标系。

正确完成后，绿色点云中的墙面/边缘应比红色更集中，拖影或弯曲减小；如果机器人完全不动，红绿应基本重合。TODO 尚未实现时，红绿也会基本重合，这是因为校正数组没有被改写。

如果 RViz 没有打开或黑屏，检查桌面显示权限、`DISPLAY`、X11/Wayland 转发和 Docker GUI 配置；这与去畸变算法本身是两个问题。

---

## 10. 提交前最小检查清单

- [ ] `Lidar_MotionCalibration()` 不再是空 TODO；
- [ ] 对每束有效激光点使用 `inverse(T_base) * T_j * p`；
- [ ] 单束分段不会除零；
- [ ] 旋转使用 slerp 或等价的最短角度插值；
- [ ] 保留 `frame_base_pose` 为整帧第一束激光的位姿；
- [ ] 距离、角度和采样时间索引一致；
- [ ] `champion_nav_msgs` 已先构建并 source；
- [ ] 使用 `rosbag play --clock laser.bag`；
- [ ] RViz 中绿色点云比红色点云更集中，或在静止情况下两者重合。
