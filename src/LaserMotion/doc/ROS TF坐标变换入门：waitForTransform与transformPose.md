# ROS TF 坐标变换入门：`waitForTransform()` 与 `transformPose()`

本文面向刚接触 ROS 的同学，解释激光去畸变代码中的两行 TF 调用：

```cpp
tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5));
tf_->transformPose("/odom", robot_pose, odom_pose);
```

它们的目标是：**查询时刻 `dt` 雷达在 `odom` 坐标系中的位置和姿态。**

相关代码可见：`../LaserUndistortion_ws/src/LaserUndistortion/src/LidarMotionUndistortion.cpp` 中的 `getLaserPose()`。

---

## 1. 先建立直觉：坐标系就是“描述位置时所选的参照物”

同一个物体的位置，换一个参照物，数值就不同。

例如，教室中有一台移动机器人：

```text
局部里程计坐标系：odom
        └── 机器人底盘坐标系：base_link
                 └── 雷达坐标系：base_laser
```

```text
odom
  │  机器人行驶时，这段关系随时间变化
  ▼
base_link
  │  雷达固定安装在车上，这段关系通常固定
  ▼
base_laser
```

- `odom`：局部里程计参考坐标系，可理解为机器人开始运动时建立的“场地参考”；它通常连续，但会因里程计累计误差而漂移；
- `base_link`：机器人底盘中心的坐标系，会随车移动、转动；

> `odom` 不是全局地图坐标系 `map`。若系统提供建图或全局定位，常见的完整链路是 `map → odom → base_link`。本作业没有使用 `map`，因此使用 `odom` 作为显示和处理的参考坐标系。
- `base_laser`：雷达自身的坐标系，雷达测得的激光点最初都在这个坐标系里。

例如，雷达测到正前方 $2\,\mathrm{m}$ 的障碍物，在雷达系中可写成：

$$
p_{laser}=
\begin{bmatrix}
2 \\
0 \\
0
\end{bmatrix}
$$

但若希望在 RViz 中显示它相对于场地的位置，就要转换到 `odom` 坐标系，得到 $p_{odom}$。

---

## 2. TF 是什么？

TF（Transform）是 ROS 中维护**坐标系之间关系**的系统。它持续收集各节点发布的坐标变换，按时间保存到缓冲区中。

一条变换可以理解为：

> 在某一个时刻，子坐标系的原点在哪里、坐标轴朝向哪里。

以：

```text
odom → base_link
```

为例，它记录的是机器人底盘在 `odom` 中的：

- 平移：$(x,y,z)$；
- 旋转：通常用四元数 $(q_x,q_y,q_z,q_w)$ 表示。

TF 能将多个坐标变换自动串起来。例如已知：

$$
T_{odom\leftarrow base}
$$

和：

$$
T_{base\leftarrow laser}
$$

就能自动得到：

$$
T_{odom\leftarrow laser}
=
T_{odom\leftarrow base}
\cdot T_{base\leftarrow laser}
$$

因此，使用 TF 时一般不需要自己写矩阵连乘。

---

## 3. 这段代码完整在做什么？

原始逻辑可以简化为：

```cpp
tf::Stamped<tf::Pose> robot_pose;
robot_pose.setIdentity();
robot_pose.frame_id_ = "/base_laser";
robot_pose.stamp_ = dt;

tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5));
tf_->transformPose("/odom", robot_pose, odom_pose);
```

逐句翻译：

1. 创建一个位于 `base_laser` 原点、没有旋转的虚拟位姿 `robot_pose`；
2. 指定该虚拟位姿的时间是 `dt`；
3. 等待 TF 准备好 `dt` 时刻从 `base_laser` 到 `odom` 的变换；
4. 用该变换把雷达原点变换到 `odom`，结果存进 `odom_pose`；
5. 因为变换的是“雷达原点 + 单位朝向”，结果自然就是雷达在 `odom` 中的位置和姿态。

最终得到：

$$
\texttt{odom\_pose}=T_{odom\leftarrow laser}(dt)
$$

---

## 4. `waitForTransform()` 详细解释

调用：

```cpp
tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5));
```

函数原型可理解为：

```cpp
waitForTransform(target_frame, source_frame, time, timeout)
```

四个参数含义如下：

| 参数 | 此处传入值 | 含义 |
|---|---|---|
| `target_frame` | `"/odom"` | **目标坐标系**：希望把结果表达在哪个坐标系中。 |
| `source_frame` | `"/base_laser"` | **源坐标系**：当前数据原本属于哪个坐标系。 |
| `time` | `dt` | 要查询哪一个历史时刻的坐标关系。 |
| `timeout` | `ros::Duration(0.5)` | 最长等候 $0.5\,\mathrm{s}$。 |

所以整句应读为：

> “请等待最多 0.5 秒，直到你能够提供 **`dt` 时刻，把 `base_laser` 中的数据变换到 `odom` 中** 所需要的 TF。”

### 4.1 为什么要等待？

ROS 中，激光消息和 TF 消息来自不同节点、不同话题，网络传输和回调执行也有先后顺序。

可能先收到一帧激光：

```text
t = 10.000 s：收到第一束激光对应的 scan
```

但 `t=10.000 s` 的里程计 TF 还没有进入当前节点的 TF 缓冲区：

```text
t = 10.003 s：收到对应时刻的 TF
```

若不等待就立刻查询，TF 会报“没有这个时间的变换”。因此先等待一小段时间，让 TF 缓冲区有机会收到所需记录。

### 4.2 为什么不能传“当前时间”？

`dt` 是某一束激光的**实际采样时刻**。机器人运动时，雷达在 $t_0$ 和 $t_1$ 的位置、姿态不同：

```text
第一束激光 t0 ── 雷达位姿 A
最后一束激光 t1 ── 雷达位姿 B
```

如果错误地传入当前时间，而不是该束激光的时间，就会把激光点放到错误的雷达位姿，运动去畸变会失败。

### 4.3 `waitForTransform()` 成功后返回什么？

在 ROS Noetic 的 tf（TF1）中，该函数返回 `bool`：

```cpp
true   // 指定变换已可查询
false  // 等待超时，指定时刻的变换仍不存在
```

因此工程代码应检查结果：

```cpp
if (!tf_->waitForTransform("/odom", "/base_laser", dt,
                           ros::Duration(0.5)))
{
    ROS_WARN("TF not ready");
    return;
}
```

> `waitForTransform()` **只负责确认/等待变换可用，不会把任何点或位姿自动转换。** 真正的转换由后续 `transformPose()` 完成。

---

## 5. `transformPose()` 详细解释

调用：

```cpp
tf_->transformPose("/odom", robot_pose, odom_pose);
```

函数原型可理解为：

```cpp
transformPose(target_frame, input_pose, output_pose)
```

参数含义：

| 参数 | 此处传入值 | 含义 |
|---|---|---|
| `target_frame` | `"/odom"` | 希望结果位姿表达在哪个坐标系。 |
| `input_pose` | `robot_pose` | 待转换的输入位姿；它自身带有 `frame_id_` 和 `stamp_`。 |
| `output_pose` | `odom_pose` | 用于接收变换结果。 |

输入 `robot_pose` 的关键设定是：

```cpp
robot_pose.setIdentity();       // 位置 (0,0,0)，姿态为无旋转
robot_pose.frame_id_ = "/base_laser";
robot_pose.stamp_ = dt;
```

因此它表示：

> “在时刻 `dt`，`base_laser` 坐标系原点处，一个与 `base_laser` 坐标轴方向完全一致的位姿。”

TF 计算：

$$
\texttt{odom\_pose}
=
T_{odom\leftarrow laser}(dt)
\cdot I
$$

其中 $I$ 是单位位姿。任何变换乘上单位位姿仍是它自己，因此：

$$
\texttt{odom\_pose}=T_{odom\leftarrow laser}(dt)
$$

这正是“雷达在 `odom` 中的位置和姿态”。

### 5.1 `odom_pose` 中具体有什么？

转换成功后：

```cpp
odom_pose.getOrigin()
```

得到雷达原点在 `odom` 中的位置：

$$
(x_{laser},y_{laser},z_{laser})
$$

```cpp
odom_pose.getRotation()
```

得到雷达相对于 `odom` 的姿态四元数：

$$
(q_x,q_y,q_z,q_w)
$$

在二维地面机器人中，通常最关心四元数转换出的偏航角 yaw：

```cpp
double roll, pitch, yaw;
tf::Matrix3x3(odom_pose.getRotation()).getRPY(roll, pitch, yaw);
```

其中 `yaw` 就是雷达在 `odom` 平面内的朝向。

---

## 6. 为什么 `target_frame` 都是 `odom`？方向会不会写反？

不会。可以记住一个实用口诀：

> `transformPose("目标坐标系", “原始坐标系中的数据”, 输出)`

此处：

```cpp
tf_->transformPose("/odom", robot_pose, odom_pose);
```

可以读作：

```text
把 base_laser 中的 robot_pose，转换成 odom 中的 odom_pose
```

即：

```text
base_laser 里的数据  ─────→  odom 里的数据
```

对应数学表达为：

$$
p_{odom}=T_{odom\leftarrow laser}\,p_{laser}
$$

注意：TF 图中常写成：

```text
odom → base_laser
```

这表示 TF 树中 `odom` 是父坐标系、`base_laser` 是子坐标系；而对一个点进行坐标转换时，使用的是将 `base_laser` 中的点表达成 `odom` 中点的变换：

$$
T_{odom\leftarrow laser}
$$

初学时容易把“TF 树的父子关系”和“点转换的箭头方向”混在一起。只需始终从数据角度问自己：**我的输入点现在在哪个坐标系？我希望输出点在哪个坐标系？**

---

## 7. 用本作业的 `laser.bag` 举例

`../bag/laser.bag` 记录了：

```text
odom → base_link → base_laser
```

对应关系：

| 关系 | TF 的直接来源 | 是否随时间变化 |
|---|---|---|
| `odom → base_link` | `/tf` | 是，机器人在运动。 |
| `base_link → base_laser` | `/tf` | 否，雷达固定安装在底盘上。 |

本 bag 也发布了 `/odom`（`nav_msgs/Odometry`）消息。它与 `odom → base_link` 表示相同类型的里程计信息，适合查看和验证机器人位姿；但 `tf::TransformListener` **不会读取 `/odom` 话题来构建 TF 树**。`waitForTransform()` 能够查询这条坐标关系的直接数据来源必须是 `/tf`。

其中雷达安装外参为：

```text
base_link → base_laser
translation = (0.210, 0.000, 0.125) m
quaternion  = (0, 0, 1, ≈0)
```

当执行：

```cpp
tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5));
tf_->transformPose("/odom", robot_pose, odom_pose);
```

TF 会自动组合：

$$
T_{odom\leftarrow laser}(dt)
=
T_{odom\leftarrow base}(dt)
\cdot T_{base\leftarrow laser}
$$

你不用手动读取 `/odom` 再计算矩阵相乘。

---

## 8. 一个具体数字例子

假设时刻 `dt`，里程计给出底盘在 `odom` 中：

```text
位置：(x, y) = (1.0, 2.0) m
偏航角：yaw = 90°
```

而雷达固定安装在车体前方 $0.21\,\mathrm{m}$。由于机器人朝向 $90^\circ$，车头正前方在 `odom` 中指向 $y$ 正方向，因此雷达原点位置为：

$$
x_{laser}=1.0+0.21\cos(90^\circ)=1.0
$$

$$
y_{laser}=2.0+0.21\sin(90^\circ)=2.21
$$

如果雷达外参还包含相对底盘的 $180^\circ$ 旋转，则雷达朝向是：

$$
yaw_{laser}=90^\circ+180^\circ=270^\circ
$$

这就是 `odom_pose` 中应得到的位置和姿态。TF 自动完成这一计算。

---

## 9. 常见报错与排查顺序

### 9.1 `waitForTransform()` 超时

常见原因：

1. 没有节点发布 `/tf`；
2. TF 树不连通，例如缺少 `base_link → base_laser`；
3. `dt` 的时间戳超出 TF 缓冲区保存范围；
4. bag 回放时 `/clock`、`use_sim_time` 或回放顺序配置不正确；
5. 坐标系名称不一致，例如代码写 `base_laser`，实际消息是 `laser`。

排查命令：

```bash
rosrun tf tf_echo odom base_laser
rosrun tf view_frames
rostopic echo -n 1 /tf
```

若回放 bag，请在**启动依赖仿真时间的节点之前**设置 `use_sim_time`，再从 bag 所在目录执行回放：

```bash
rosparam set use_sim_time true
cd /home/zachary/workspace/ROS_NOETIC/src/LaserMotion/bag
rosbag play --clock laser.bag
```

`rosbag play --clock` 会发布 `/clock`。节点启动后才设置 `use_sim_time` 可能导致节点已经按真实系统时间初始化，容易出现时间不匹配；因此更推荐把 `use_sim_time:=true` 写入启动文件，或先设置参数、再启动节点。

### 9.2 `transformPose()` 抛出 `TransformException`

即使刚刚 `waitForTransform()` 返回成功，实际转换也可能因时间、坐标系配置或 TF 缓冲区状态异常失败。因此要使用 `try/catch`：

```cpp
try
{
    tf_->transformPose("/odom", robot_pose, odom_pose);
}
catch (const tf::TransformException& exception)
{
    ROS_ERROR("Cannot transform laser pose: %s", exception.what());
    return;
}
```

### 9.3 坐标系前面的 `/` 要不要写？

ROS1 中常能看到 `"/odom"` 和 `"odom"` 两种写法。当前项目会统一去除前导 `/`，内部使用 `odom`、`base_laser`，以避免不同 ROS 名称解析规则带来困扰。

学习旧代码时，`"/odom"` 和 `"odom"` 在常见的全局坐标系用法中通常指向同一帧；写新代码时，更推荐统一使用不带 `/` 的 `"odom"`、`"base_laser"`。

---

## 10. 最终记忆卡片

```text
waitForTransform(target, source, time, timeout)
= 等待并确认：time 时刻能否把 source 中的数据转换到 target。

transformPose(target, input, output)
= 真正执行转换：把 input.frame_id_ 中的位姿转换到 target。

本例：
source = base_laser
 target = odom
   time = dt（激光实际采样时刻）

输出：雷达在 odom 中的位置和姿态。
```

如果只记一句话：

> **先用 `waitForTransform()` 确认 `dt` 时刻的 TF 可用，再用 `transformPose()` 把 `base_laser` 中的单位位姿转到 `odom`，得到雷达在 `odom` 中的完整位姿。**
