# 从激光去畸变到 ICP 实现路线

本文给出当前 LaserMotion 作业从**单帧激光去畸变**扩展到 **ICP 激光里程计**，再到 **扫描匹配局部地图**的渐进实现路线。

建议严格按本文的阶段顺序实现和验证：先让每一步独立正确，再进入下一步。不要在去畸变尚未验证时就直接接入 ICP。

相关文件：

- 去畸变节点：`../LaserUndistortion_ws/src/LaserUndistortion/src/LidarMotionUndistortion.cpp`
- 去畸变 TODO 讲解：[激光数据处理流程讲解.md](激光数据处理流程讲解.md)
- ICP 数学原理：[ICP匹配原理与SVD讲解.md](ICP匹配原理与SVD讲解.md)
- ICP 知识体系：[ICP点线体系化学习指南.md](ICP点线体系化学习指南.md)

---

## 0. 当前已搭好的开发框架

框架已经放在 `LaserUndistortion` 包中。你不需要再处理扫描数据校验、分段边界、点云构造、首帧缓存、里程计初值、降采样、质量门控、RViz 诊断话题或 CMake/PCL 依赖。

### 0.1 两处需要你手写的核心 TODO

| TODO | 文件与位置 | 你的任务 |
|---|---|---|
| 运动去畸变 | `../LaserUndistortion_ws/src/LaserUndistortion/src/LidarMotionUndistortion.cpp` 的 `Lidar_MotionCalibration()` | 利用分段首尾 TF 插值得到每束激光瞬时位姿，并统一变换到本帧第一束的 `base_laser` 坐标系。 |
| ICP 配准 | `../LaserUndistortion_ws/src/LaserUndistortion/src/icp_odometry.cpp` 的 `PclIcpEstimator::estimate()` | 以当前帧为 source、上一帧为 target，使用 odom 相对位姿初值，计算 `$T_{previous\leftarrow current}$`。 |

`Lidar_MotionCalibration()` 的逐行参考代码位于：[激光数据处理流程讲解.md](激光数据处理流程讲解.md)。

### 0.2 框架已经替你完成的工作

```text
ChampionNavLaserScan
        │
        ▼
输入校验、时间戳/帧号检查、无效点过滤
        │
        ▼
Lidar_Calibration()：5 ms 不重叠分段、TF 查询、失败原子化处理
        │
        ▼
TODO 1：Lidar_MotionCalibration()
        │
        ▼
corrected_local_cloud（当前帧局部 ICP 输入）
        │
        ▼
IcpOdometry：有限点过滤、VoxelGrid、首帧缓存、odom 初值、质量门控
        │
        ▼
TODO 2：PclIcpEstimator::estimate()
        │
        ▼
ICP RViz 诊断点云
```

### 0.3 新增话题与 RViz 验证层

原有可视化保持不变：

| 话题 | 颜色 | 含义 |
|---|---|---|
| `/laser_motion/raw_cloud` | 红色 | 未去畸变扫描，按起始 TF 放入 `odom`。 |
| `/laser_motion/corrected_cloud` | 绿色 | 去畸变扫描，按起始 TF 放入 `odom`。在 TODO 1 未实现时与红色重合。 |

为 ICP 新增的诊断话题已经写入 RViz 配置，但默认关闭，避免影响当前作业观察：

| 话题 | 建议颜色 | 何时出现 |
|---|---|---|
| `/laser_motion/corrected_local_cloud` | 自选 | 每帧都有；是 ICP 的局部输入数据。 |
| `/laser_motion/icp/previous_cloud` | 蓝色 | 从第二个合格扫描开始。上一帧 target 放在 `odom` 中。 |
| `/laser_motion/icp/current_before` | 橙色 | 从第二个合格扫描开始。当前帧按照 odom 初值放在 `odom` 中。 |
| `/laser_motion/icp/current_aligned` | 青色 | 仅当 TODO 2 返回的结果通过收敛、误差、二维运动与 odom 偏差检查时发布。 |

在 RViz 左侧勾选 `ICP Previous Cloud (blue)`、`ICP Current Before (orange)` 和 `ICP Current Aligned (cyan)` 即可查看。ICP 未实现时，前两个诊断点云会出现，`current_aligned` 不会出现，这是预期行为。

### 0.4 可调参数

启动文件 `../LaserUndistortion_ws/src/LaserUndistortion/launch/LaserUndistortion.launch` 已暴露如下参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `start_rviz` | `true` | 设为 `false` 可在无图形环境只启动节点。 |
| `scan_topic` | `champion_scan` | 输入扫描话题。 |
| `odom_frame` / `laser_frame` | `odom` / `base_laser` | TF 查询使用的坐标系。 |
| `tf_timeout_sec` | `0.5` | 等待指定时刻 TF 的超时。 |
| `motion_segment_sec` | `0.005` | 去畸变分段时长，单位秒。 |
| `voxel_leaf_size` | `0.05` | ICP 输入的体素降采样尺寸，单位米；`0` 表示关闭。 |
| `minimum_icp_points` | `30` | 过滤后少于此点数，不调用 ICP。 |
| `maximum_icp_fitness` | `0.10` | ICP fitness 上限。 |
| `maximum_translation_correction` | `0.50` | ICP 相对 odom 初值允许的最大平移修正，单位米。 |
| `maximum_yaw_correction_degrees` | `30.0` | ICP 相对 odom 初值允许的最大航向修正，单位度。 |

例如，不启动 RViz：

```bash
roslaunch LaserUndistortion LaserUndistortion.launch start_rviz:=false
```

---

## 1. 先建立完整概念图

当前代码处理的是**一帧激光扫描内部**的时间不一致问题；ICP 处理的是**两帧点云或点云与地图之间**的位姿关系问题。

```text
原始 LaserScan
    │
    │ 每束激光的采样时刻不同；机器人可能正在运动
    ▼
基于 TF / odom 的运动去畸变
    │
    │ 统一到本帧第一束激光的时刻
    ▼
corrected_cloud（绿色去畸变点云）
    │
    │ 当前帧与上一帧 / 子地图进行几何配准
    ▼
ICP
    │
    ▼
更准确的相对位姿 / map 中位姿
    │
    ▼
激光里程计、局部建图、定位
```

两者对比：

| 内容 | 当前去畸变 TODO | ICP |
|---|---|---|
| 处理对象 | 同一帧扫描内部的各束激光 | 两帧点云，或当前帧与地图 |
| 已知信息 | 每束时刻对应的 TF / odom 位姿 | 两组点云的几何形状 |
| 要解决的问题 | 扫描过程中运动造成的墙面弯曲、重影 | 估计当前帧相对上一帧/地图的位姿 |
| 位姿来源 | TF 插值 | 最近邻对应点 + 迭代优化 |
| 是否需要迭代 | 否 | 是 |

可以记成一句话：

```text
去畸变：我知道机器人怎么动，所以把点直接变回来。
ICP：我不知道机器人精确怎么动，所以用两组点云反推位姿。
```

---

## 2. 当前绿色 `corrected_cloud` 的坐标意义

在 `Lidar_MotionCalibration()` 中，每束激光经过补偿后都会统一表达在：

```text
本帧第一束激光采集时刻的 base_laser 坐标系
```

数学形式为：

$$
p_{base,j}
=
T_{odom\leftarrow laser}(t_0)^{-1}
\cdot T_{odom\leftarrow laser}(t_j)
\cdot p_{laser,j}
$$

其中：

- $p_{laser,j}$：第 $j$ 束激光在它采集瞬间的雷达坐标系中的点；
- $T_{odom\leftarrow laser}(t_j)$：第 $j$ 束激光时刻，雷达到 `odom` 的位姿；
- $T_{odom\leftarrow laser}(t_0)$：本帧第一束激光时刻的雷达位姿；
- $p_{base,j}$：去畸变后、统一到本帧起始雷达坐标系的点。

当前的 `PublishCloudInOdom()` 再将这些点使用起始位姿变换到 `odom`，供 RViz 显示。

因此可以区分两个概念：

```text
去畸变局部点云：本帧起始时刻的 base_laser 坐标系
绿色 RViz 点云：经过起始位姿转换后的 odom 坐标系
```

后续 ICP 最干净的输入是前者，即**去畸变完成、但仍在本帧局部 `base_laser` 坐标系中的点云**。

---

## 3. 实现总路线

建议分为四个阶段：

```text
阶段 0：完成并验证单帧运动去畸变
   │
   ▼
阶段 1：构造本地去畸变点云 corrected_scan_local
   │
   ▼
阶段 2：帧间 ICP（当前帧 ↔ 上一帧）
   │
   ▼
阶段 3：扫描到局部地图 ICP（当前帧 ↔ submap）
   │
   ▼
阶段 4：关键帧、全局地图、回环与图优化（进阶）
```

阶段 0 至阶段 2 是最适合当前作业继续扩展的最小闭环。

---

# 阶段 0：完成并验证单帧去畸变

## 0.1 要做什么

完成 `LidarMotionUndistortion.cpp` 中的：

```cpp
void Lidar_MotionCalibration(...)
```

可手抄的代码和逐行讲解已在：[激光数据处理流程讲解.md](激光数据处理流程讲解.md) 的“完成 `Lidar_MotionCalibration()` TODO”章节中。

## 0.2 必须验证什么

在 RViz 中观察：

```text
红色：/laser_motion/raw_cloud
绿色：/laser_motion/corrected_cloud
```

预期结果：

| 机器人状态 | 正确现象 |
|---|---|
| 机器人静止 | 红绿点云基本重合 |
| 机器人平移/转动 | 绿色墙面更集中、重影更少 |
| TODO 尚未完成 | 红绿点云基本完全重合 |

## 0.3 通过本阶段的条件

- [ ] `Lidar_MotionCalibration()` 已不再是空 TODO；
- [ ] 第一个点和最后一个点的插值比例正确；
- [ ] 静止时，校正前后点云基本不变；
- [ ] 运动时，绿色点云的直线墙面比红色更规整；
- [ ] 不出现 TF 连续报错、NaN 点或数组越界。

---

# 阶段 1：构造去畸变后的局部点云

## 1.1 为什么要单独构造局部点云

当前 `PublishCloudInOdom()` 同时承担了两件事：

1. 将极坐标 `ranges/angles` 转成点云；
2. 根据起始位姿将点云放到 `odom` 中。

但 ICP 的输入最好保持在每帧自己的局部雷达坐标系中。这样 ICP 模块只关心：

```text
当前帧局部点云如何与上一帧局部点云对齐？
```

而不混入 `odom` 可视化逻辑。

## 1.2 建议新增的辅助函数

可以新增一个专门构造局部点云的函数：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr BuildCloudInLaser(
    const std::vector<double>& ranges,
    const std::vector<double>& angles)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    cloud->points.reserve(ranges.size());

    for (std::size_t i = 0; i < ranges.size(); ++i)
    {
        // 与前面的数据清洗逻辑保持一致。
        if (ranges[i] < 0.05 ||
            !std::isfinite(ranges[i]) ||
            !std::isfinite(angles[i]))
        {
            continue;
        }

        // 极坐标 -> 当前帧起始 base_laser 坐标系中的直角坐标。
        pcl::PointXYZ point;
        point.x = static_cast<float>(ranges[i] * std::cos(angles[i]));
        point.y = static_cast<float>(ranges[i] * std::sin(angles[i]));
        point.z = 0.0F;

        cloud->points.push_back(point);
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}
```

> 上述 `ranges/angles` 必须是调用 `Lidar_Calibration()` **之后**的数据。因为这时每个点已经被表达在本帧第一束激光的 `base_laser` 坐标系中。

## 1.3 应放在回调的哪个位置

推荐顺序如下：

```cpp
// 1. 原始数据副本，供红色 RViz 点云显示。
const std::vector<double> raw_ranges = ranges;
const std::vector<double> raw_angles = angles;

// 2. 原地去畸变：ranges / angles 现在代表本帧起始雷达系中的扫描。
Lidar_Calibration(ranges, angles, startTime, endTime, tf_);

// 3. 为 ICP 构造局部去畸变点云。
auto corrected_scan_local = BuildCloudInLaser(ranges, angles);

// 4. 把局部点云交给后续 ICP 模块。
// ProcessIcp(corrected_scan_local, frame_base_pose);

// 5. 发布红绿点云用于 RViz。
PublishCloudInOdom(raw_ranges, raw_angles,
                   frame_base_pose, startTime, raw_cloud_pub_);
PublishCloudInOdom(ranges, angles,
                   frame_base_pose, startTime, corrected_cloud_pub_);
```

## 1.4 通过本阶段的条件

- [ ] `corrected_scan_local` 的点全部位于当前帧局部雷达坐标系；
- [ ] 不重复将 `frame_base_pose` 施加给 ICP 输入；
- [ ] 点云中无 NaN / Inf；
- [ ] 点数足够时才进入后续 ICP；
- [ ] RViz 的绿色点云显示仍然正常。

---

# 阶段 2：帧间 ICP（Scan-to-Scan）

这是第一个值得实现的 ICP 版本。它将当前帧点云与上一帧点云进行匹配。

## 2.1 数据关系

```text
previous_scan：上一帧去畸变局部点云
current_scan ：当前帧去畸变局部点云

ICP 的 source：current_scan
ICP 的 target：previous_scan
```

ICP 的输出变换为：

$$
p_{prev} = T_{prev\leftarrow curr} \cdot p_{curr}
$$

也就是说，`T_prev_curr` 将当前帧坐标系中的点变换到上一帧坐标系。

```text
当前帧 current_scan
      │
      │ T_prev←curr
      ▼
上一帧 previous_scan
```

## 2.2 一定要使用 odom 初始猜测

ICP 是局部优化算法。如果初始位置距离正确解太远，可能：

- 无法收敛；
- 收敛到错误位置；
- 把形状相似但实际无关的墙面匹配在一起。

当前系统已经有 odom / TF，所以应该先计算两帧之间的 odom 相对位姿作为初值。

设：

- $T_{odom\leftarrow prev}$：上一帧雷达坐标系到 `odom` 的位姿；
- $T_{odom\leftarrow curr}$：当前帧雷达坐标系到 `odom` 的位姿。

则 ICP 初值为：

$$
T_{prev\leftarrow curr}^{init}
=
T_{odom\leftarrow prev}^{-1}
\cdot T_{odom\leftarrow curr}
$$

代码层面的概念：

```cpp
const Eigen::Matrix4f initial_guess =
    previous_odom_pose.inverse() * current_odom_pose;
```

> 实际实现时要统一矩阵类型和坐标系方向。上式的关键不是类型，而是变换方向：**当前帧到上一帧**。

## 2.3 最小 PCL ICP 代码骨架

需要在 CMake 中加入 PCL 的 registration 组件，并包含：

```cpp
#include <pcl/registration/icp.h>
```

核心调用结构：

```cpp
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

// source 是待移动的当前帧。
icp.setInputSource(current_scan);

// target 是保持不动的上一帧。
icp.setInputTarget(previous_scan);

// 以下参数要结合实际激光数据继续调整。
icp.setMaximumIterations(40);
icp.setMaxCorrespondenceDistance(0.5);
icp.setTransformationEpsilon(1e-6);
icp.setEuclideanFitnessEpsilon(1e-5);

// aligned 中的点已落在 previous_scan 的坐标系中。
pcl::PointCloud<pcl::PointXYZ> aligned;
icp.align(aligned, initial_guess);

if (!icp.hasConverged())
{
    ROS_WARN("ICP did not converge");
    return;
}

const Eigen::Matrix4f T_prev_curr = icp.getFinalTransformation();
const double fitness = icp.getFitnessScore();
```

### `source` 与 `target` 千万不要记反

```text
source = current_scan  ：当前帧，需要被变换

target = previous_scan ：上一帧，作为参考
```

因此，`getFinalTransformation()` 的语义是：

```text
current_scan 坐标系 → previous_scan 坐标系
```

## 2.4 第一帧怎么处理

第一帧不存在 `previous_scan`，不能执行 ICP：

```cpp
if (!has_previous_scan_)
{
    previous_scan_ = current_scan;
    previous_odom_pose_ = current_odom_pose;
    has_previous_scan_ = true;
    return;
}
```

以后每一帧成功处理后，更新历史状态：

```cpp
previous_scan_ = current_scan;
previous_odom_pose_ = current_odom_pose;
```

## 2.5 帧间 ICP 的完整状态流程

```text
收到当前 LaserScan
        │
        ▼
完成运动去畸变
        │
        ▼
构造 current_scan（当前帧局部点云）
        │
        ├── 没有上一帧？ ── 是 ──► 保存当前帧，等待下一帧
        │
        ▼ 否
取得 current odom pose 和 previous odom pose
        │
        ▼
计算 current → previous 的 odom 初值
        │
        ▼
ICP(current_scan, previous_scan, initial_guess)
        │
        ├── ICP 不可信？ ── 是 ──► 丢弃本次 ICP 结果，保留 odom 预测
        │
        ▼ 否
保存 ICP 输出相对位姿
        │
        ▼
更新 previous_scan / previous_pose
```

## 2.6 通过本阶段的条件

- [ ] 第一帧只缓存，不运行 ICP；
- [ ] `source` 是当前帧，`target` 是上一帧；
- [ ] 使用 odom 相对位姿作为 `initial_guess`；
- [ ] 能检查 `hasConverged()`；
- [ ] 能检查 `fitness score`；
- [ ] 能在 RViz 中对比 ICP 前后的当前帧；
- [ ] 当前帧对齐后，和上一帧的墙面、障碍物边缘明显更重合。

---

# 阶段 2.7：在 RViz 中验证帧间 ICP

建议额外发布三份点云：

| 话题示例 | 坐标系 | 作用 |
|---|---|---|
| `/icp/previous_cloud` | `odom` 或 `map` | 上一帧参考点云 |
| `/icp/current_before` | `odom` 或 `map` | 当前帧按 odom 初值放置的点云 |
| `/icp/current_aligned` | `odom` 或 `map` | ICP 对齐后的当前帧点云 |

预期画面：

```text
ICP 前：

上一帧：    ███████████
当前帧：      ███████████

ICP 后：

上一帧：    ███████████
当前帧：    ███████████
```

若 `current_aligned` 反而与 `previous_cloud` 偏差更大，优先检查：

1. `source` 与 `target` 是否写反；
2. 初始猜测矩阵的方向是否写反；
3. 点云是否已经去畸变；
4. 两帧是否有足够重叠区域；
5. `max correspondence distance` 是否过小或过大。

---

# 阶段 3：扫描到局部地图 ICP（Scan-to-Map）

只匹配上一帧的问题是：上一帧的小误差会逐帧传递，时间越久漂移越明显。

下一步可以让当前帧对齐由多帧关键帧组成的局部子地图：

```text
关键帧 1 ─┐
关键帧 2 ─┼──► 局部子地图 submap
关键帧 3 ─┘            ▲
                       │ ICP
                       │
                 当前去畸变点云
```

## 3.1 scan-to-map 的输入和输出

```text
source：当前帧去畸变局部点云

target：机器人附近若干关键帧组成的 submap

输出：当前帧在 map 坐标系中的精确位姿
```

设：

- $T_{map\leftarrow odom}$：当前的地图到里程计修正；
- $T_{odom\leftarrow laser}$：原始里程计提供的当前雷达位姿。

那么，当前帧的 map 位姿预测为：

$$
T_{map\leftarrow laser}^{init}
=
T_{map\leftarrow odom}
\cdot T_{odom\leftarrow laser}
$$

将它作为 scan-to-map ICP 的初始猜测。

ICP 匹配成功后获得：

$$
T_{map\leftarrow laser}^{ICP}
$$

再更新 map 到 odom 的修正：

$$
T_{map\leftarrow odom}
=
T_{map\leftarrow laser}^{ICP}
\cdot
\left(T_{odom\leftarrow laser}\right)^{-1}
$$

## 3.2 推荐 TF 树

不要让 ICP 与现有里程计同时发布同一条 `odom → base_laser` TF。推荐 TF 关系是：

```text
map
 │
 └── map → odom              ICP / SLAM 模块发布
       │
       └── odom → base_laser  现有里程计 / 仿真 TF 发布
```

各边的职责：

| TF | 发布者 | 特点 |
|---|---|---|
| `odom → base_laser` | 现有里程计或仿真 | 短期平滑连续，但长期有漂移 |
| `map → odom` | ICP / SLAM 模块 | 用环境几何修正长期漂移 |
| `map → base_laser` | TF 自动串联 | 得到最终的全局雷达位姿 |

## 3.3 关键帧策略

不要保存所有点云。可以在满足以下任一条件时插入关键帧：

```text
相对上一关键帧平移超过 0.3 m
或
相对上一关键帧旋转超过 10°
或
当前帧与上一关键帧的重叠明显降低
```

关键帧处理：

```text
当前帧局部点云
        │
        ▼
使用当前 map 位姿变换到 map 坐标系
        │
        ▼
作为关键帧保存
        │
        ▼
加入附近局部子地图
```

## 3.4 局部子地图的原则

每次 ICP 不应匹配所有历史点，而应只取机器人附近的地图：

```text
全局地图
+-----------------------------------+
|                                   |
|         +---------------+         |
|         | 局部 submap    |         |
|         |    机器人      |         |
|         +---------------+         |
|                                   |
+-----------------------------------+
```

好处：

- 最近邻搜索更快；
- 降低匹配到远处相似墙面的概率；
- 对局部定位更稳定；
- 有利于实时运行。

---

# 阶段 4：二维激光项目的进阶方向

本项目的激光点始终是：

```cpp
point.z = 0.0F;
```

所以实际只需要估计二维刚体运动 $SE(2)$：

$$
T =
\begin{bmatrix}
\cos\theta & -\sin\theta & t_x \\
\sin\theta & \cos\theta & t_y \\
0 & 0 & 1
\end{bmatrix}
$$

其中未知量只有：

```text
x、y、yaw
```

而通用 `pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>` 面向三维 $SE(3)$，还会考虑：

```text
z、roll、pitch
```

对于纯二维共面点云，这三个量缺乏约束，可能带来数值不稳定。因此建议的学习顺序是：

```text
1. 用 PCL 标准 ICP 跑通 scan-to-scan 全流程。
2. 学习二维点到点 ICP，限制优化变量为 x、y、yaw。
3. 学习点到线 ICP（point-to-line ICP）。
4. 再学习关键帧、scan-to-map、回环检测、位姿图优化。
```

点到线 ICP 对走廊、墙壁等二维激光场景通常更合适，因为它最小化的是“点到墙所在直线”的距离，而不强行将点匹配到另一个离散点。

---

# 4. ICP 前的点云预处理

绿色点云去畸变后仍不应不加处理地直接送入 ICP。

推荐管线：

```text
corrected_scan_local
        │
        ▼
去除无效点
        │
        ▼
距离范围裁剪
        │
        ▼
体素降采样 VoxelGrid
        │
        ▼
可选：离群点滤波
        │
        ▼
ICP
```

## 4.1 最少需要的质量检查

| 检查项 | 为什么需要 |
|---|---|
| 点数是否足够 | 点过少时，几何约束不足，ICP 结果不可靠。 |
| `hasConverged()` | 判断 ICP 是否满足自身停止条件。 |
| `getFitnessScore()` | 判断最终对应误差是否足够小。 |
| 最大对应距离 | 防止把远处无关结构配对。 |
| 与 odom 初值的差异 | 防止 ICP 跳到明显不合理的位置。 |
| 降采样 | 减少重复点，提高速度与数值稳定性。 |

特别注意：

> `icp.hasConverged()` 只表示迭代停止，不代表匹配结果一定正确。

因此必须联合判断：

```text
是否收敛
+ fitness score 是否足够小
+ 位姿修正是否合理
+ 有效点数是否足够
```

---

# 5. 推荐的代码模块划分

初学时可以先把逻辑放在一个节点中；当流程正确后，再逐步拆分职责。

```text
LidarMotionCalibrator
    ├── 接收 ChampionNavLaserScan
    ├── 运动去畸变
    ├── 生成 corrected_scan_local
    └── 发布 raw / corrected RViz 点云

IcpOdometry
    ├── 缓存上一帧或局部子地图
    ├── 用 odom 计算 ICP 初值
    ├── 执行 ICP
    ├── 检查匹配质量
    ├── 累积 ICP 位姿
    └── 发布 map → odom TF

LocalMapManager
    ├── 管理关键帧
    ├── 构建局部 submap
    ├── 体素降采样
    └── 给 ICP 提供 target 点云
```

建议：

1. **先在同一个 `.cpp` 中实现 Scan-to-Scan ICP**，方便调试；
2. 确认 source/target/初值方向全正确后；
3. 再抽取 `IcpOdometry` 类；
4. 最后添加 `LocalMapManager`，实现 Scan-to-Map。

---

# 6. 最小可完成目标清单

后续建议一次只完成一项，并在 RViz 中验证。

## 第一轮：去畸变

- [ ] 手抄并实现 `Lidar_MotionCalibration()`；
- [ ] 编译通过；
- [ ] bag 回放时红绿点云可见；
- [ ] 机器人运动时绿色点云更集中。

## 第二轮：ICP 输入

- [ ] 新增 `BuildCloudInLaser()`；
- [ ] 从补偿后的 `ranges/angles` 构造 `corrected_scan_local`；
- [ ] 检查点数、NaN 和 Inf；
- [ ] 保留上一帧点云和上一帧 odom 位姿。

## 第三轮：帧间 ICP

- [ ] 第一帧只缓存；
- [ ] 当前帧作为 source，上一帧作为 target；
- [ ] 用 odom 相对位姿作为 ICP 初值；
- [ ] 检查 `hasConverged()` 和 `fitness score`；
- [ ] 发布 ICP 前后点云到 RViz；
- [ ] 确认对齐后两个扫描中的墙面重合更好。

## 第四轮：局部地图 ICP

- [ ] 实现关键帧判定；
- [ ] 构建附近局部子地图；
- [ ] 当前帧对齐到 `submap`；
- [ ] 发布 `map → odom`，不要重复发布 `odom → base_laser`；
- [ ] 在长距离运动中检查漂移改善情况。

---

# 7. 实现时最容易犯的错误

| 错误 | 现象 | 正确做法 |
|---|---|---|
| 未先去畸变就做 ICP | 匹配不稳定、墙面形变严重 | 先完成并验证绿色去畸变点云。 |
| source / target 写反 | ICP 输出变换方向混乱 | `source=current`，`target=previous/submap`。 |
| 初值方向写反 | ICP 初始位置离目标很远 | 严格写清楚 $T_{target\leftarrow source}$。 |
| 把局部点云与 odom 点云混用 | 坐标系错乱、显示跳变 | 每个点云明确记录 `frame_id` 和变换方向。 |
| 只检查 `hasConverged()` | 会接受错误局部最优解 | 再检查 fitness、位姿跳变、点数。 |
| 每帧存入全局地图 | 越跑越慢 | 使用关键帧和局部子地图。 |
| ICP 也发布 `odom → base_laser` | TF 冲突、RViz 跳变 | ICP/SLAM 只发布 `map → odom`。 |
| 直接按 3D 自由度使用共面数据 | 出现 z/roll/pitch 偏移 | 最终应采用二维 $SE(2)$ 或点到线 ICP。 |

---

# 8. 最后总结

你的推荐实现路径是：

```text
第一步：完成单帧去畸变 TODO
        │
        ▼
第二步：从校正后的 ranges / angles 构造局部点云
        │
        ▼
第三步：当前帧与上一帧做 ICP（Scan-to-Scan）
        │
        ▼
第四步：当前帧与关键帧局部地图做 ICP（Scan-to-Map）
        │
        ▼
第五步：用 map → odom 将 ICP 修正融入 ROS TF 树
```

最重要的两条原则：

```text
1. 去畸变解决一帧内部的问题，ICP 解决帧与帧/地图之间的问题。
2. 所有矩阵和点云都必须明确“从哪个坐标系变到哪个坐标系”。
```

如果坐标系语义一直写清楚，例如：

$$
T_{target\leftarrow source}
$$

那么绝大多数 ICP 实现中最常见的“方向写反”问题都可以避免。
