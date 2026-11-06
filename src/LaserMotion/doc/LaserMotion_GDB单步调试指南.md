# LaserMotion：用 GDB 单步观察激光去畸变与 ICP

先让 `rosbag` 停在第一帧激光前，再让 GDB 在回调、去畸变和 ICP 的边界依次停下。下面是这份作业最适合入门的单步调试方式。

```text
rosbag（暂停） ──恢复一次──> /champion_scan
                                  │
                                  ▼
                     scanCallback()：拿到一整帧原始数据
                                  │
                                  ▼
                Lidar_Calibration()：切成 5 ms 的小段
                                  │
                                  ▼
           Lidar_MotionCalibration()：逐束点做坐标变换
                                  │
                                  ▼
                   buildCloudInLaser()：极坐标变点云
                                  │
                                  ▼
                    IcpOdometry::process()：两帧 ICP 外框
                                  │
                                  ▼
             PclIcpEstimator::estimate()：ICP 求解 TODO
```

> 本文基于当前工程实际代码。去畸变 TODO 在 `LidarMotionUndistortion.cpp:270`；ICP TODO 在 `icp_odometry.cpp:136`。当前两处仍是占位实现：因此本指南的目标是**看懂数据如何流动、验证坐标系与变量含义**，而不是假装 ICP 已经有匹配结果。

---

## 1. 先建立调试时的三个原则

### 原则 1：一次只放行一帧激光

这个 bag 总长约 `52.96 s`，其中有 `588` 帧 `/champion_scan`。正常速度播放时，GDB 会频繁命中断点，基本无法观察变量。

因此用 `rosbag --pause` 启动，然后通过 ROS 服务恢复或暂停播放。暂停状态下，已经进入回调的那一帧依然可以在 GDB 里随意单步；下一帧不会继续涌入。

### 原则 2：把“代码单步”和“数据单步”分开

- **代码单步**：在 GDB 中用 `next`、`step`、`finish` 穿过 C++ 语句和函数；
- **数据单步**：通过暂停/恢复 `rosbag`，决定何时放行下一帧 `/champion_scan`；
- **画面单步**：GDB 放行代码后，RViz 才会收到当前帧的红绿点云，供你验证理解。

不要在 `rosbag` 高速播放时，试图只靠 GDB 的 `next` 来“冻结世界”。ROS 回调、TF 和 RViz 都是独立的数据流。

### 原则 3：先看局部坐标，再看 odom 坐标

最容易混淆的是“点云看上去都在 RViz 里”，但它们实际所在的坐标系不同：

| 观察对象 | 变量 / 话题 | 所在坐标系 | 用来回答的问题 |
|---|---|---|---|
| 原始扫描 | `ranges`、`angles` | 每束自己的瞬时 `base_laser` | 传感器实际量到了什么？ |
| 校正扫描 | `corrected_ranges`、`corrected_angles` | **本帧第一束**的 `base_laser` | 去畸变后，各束是否回到同一时刻？ |
| ICP 本地输入 | `/laser_motion/corrected_local_cloud` | 当前帧起始 `base_laser` | ICP 实际收到什么点？ |
| RViz 红/绿点云 | `/laser_motion/raw_cloud`、`/laser_motion/corrected_cloud` | `odom` | 物理结果是否直观正确？ |
| ICP 诊断点云 | `/laser_motion/icp/*` | `odom` | odom 初值与 ICP 结果是否对齐？ |

当前代码在 `LidarMotionUndistortion.cpp:207-218` 明确先构造局部 `corrected_local_cloud`，再乘以扫描起始时刻的 `T_odom_laser` 放入 `odom`。所以本地数据与 RViz 展示数据不是同一坐标系，不能直接把数值逐项比较。

---

## 2. 一次性准备 Debug 构建

当前工作空间缓存的是 `Release` 构建；Release 经常会优化掉局部变量、折叠代码行，因此不能获得可靠的逐行观察体验。调试前必须改成 `Debug`。

下面命令在仓库 Docker 环境执行。构建顺序不能颠倒：`champion_nav_msgs` 是独立 workspace，主工程必须先 source 它的 overlay。

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker

# 1. 先构建自定义消息包（保留调试符号）
./run.sh "source /opt/ros/noetic/setup.bash && \
  cd /workspace/src/LaserMotion/champion_nav_msgs && \
  catkin_make -DCMAKE_BUILD_TYPE=Debug"

# 2. 再构建去畸变 / ICP 节点（保留调试符号）
./run.sh "source /opt/ros/noetic/setup.bash && \
  source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash && \
  cd /workspace/src/LaserMotion/LaserUndistortion_ws && \
  catkin_make -DCMAKE_BUILD_TYPE=Debug"
```

验证可执行程序与调试符号是否存在：

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker
./run.sh "gdb -q -batch \
  -ex 'info line LidarMotionUndistortion.cpp:270' \
  /workspace/src/LaserMotion/LaserUndistortion_ws/devel/lib/LaserUndistortion/LaserUndistortion_node"
```

若输出包含源文件、行号和地址，说明 GDB 已能定位源码。若显示 `No line number information available`，请确认第二步真的使用了 `-DCMAKE_BUILD_TYPE=Debug`，然后重新构建。

> 调试完成后想恢复性能构建，只需把两次 `Debug` 换回 `Release` 再执行一遍。`build/`、`devel/` 与 `install/` 都是本地构建产物，不会提交进 Git。

---

## 3. 用 VS Code 图形化调试（推荐）

这套工程已经加入了可共享的 VS Code 配置：

- [`.vscode/launch.json`](../LaserUndistortion_ws/.vscode/launch.json)：`F5` 启动 GDB 节点；
- [`.vscode/tasks.json`](../LaserUndistortion_ws/.vscode/tasks.json)：`Ctrl+Shift+B` 执行 Debug 构建；
- [`debug_laser_undistortion.gdb`](../LaserUndistortion_ws/src/LaserUndistortion/scripts/debug_laser_undistortion.gdb)：终端版 GDB 的同一组推荐断点。

### 3.1 必装扩展

在宿主机 VS Code 中安装 Microsoft 的 **C/C++**（扩展 ID：`ms-vscode.cpptools`）。它提供左侧的 Run and Debug 面板、断点红点、变量树、调用栈，以及单步按钮。

然后通过 Docker 容器开发窗口打开工作空间：

1. 确保容器已经由仓库脚本启动：

   ```bash
   cd /home/zachary/workspace/ROS_NOETIC/docker
   ./run.sh
   ```

2. VS Code 按 `Ctrl+Shift+P`，选择 **Dev Containers: Attach to Running Container...**；
3. 选择 `ros_noetic_hw`；
4. 新开的 VS Code 窗口中，使用 **File → Open Folder...** 打开：

   ```text
   /workspace/src/LaserMotion/LaserUndistortion_ws
   ```

一定要在这个 **容器窗口**中按 `F5`。因为 GDB、ROS、编译后的 `LaserUndistortion_node` 都在容器内部；不要在宿主机直接调试 `/workspace/...` 路径。

> 若 VS Code 提示安装 C/C++ 扩展，选择安装到 `ros_noetic_hw` 容器中，而非只安装到本机。扩展的 C++ 语言服务和 `cppdbg` 调试器需要运行在相同容器环境。

### 3.2 图形化调试前的准备

1. `F5` 会先自动运行任务 **LaserMotion: Debug 构建节点**；也可以先按 `Ctrl+Shift+B` 手动运行它。每次改 C++ 源码后，先执行该任务再调试，保证源代码行号与二进制一致；
2. 打开 `src/LaserUndistortion/src/LidarMotionUndistortion.cpp`，在下面四处行号左侧单击，出现红色断点：

   | 断点所在处 | 作用 |
   |---|---|
   | `scanCallback()` 的函数体第一行 | 一帧 `/champion_scan` 刚进入节点 |
   | `Lidar_MotionCalibration()` 的函数体第一行 | 一个 5 ms 分段刚进入去畸变 TODO |
   | `buildCloudInLaser(corrected_ranges, corrected_angles)` 调用处 | 已校正极坐标即将变为局部点云 |
   | `icp_odometry.cpp` 的 `IcpOdometry::process()` 函数体第一行 | 当前局部点云进入 ICP 外框 |

3. **先完成下一节的 `roscore` 和 `/use_sim_time` 设置**，再打开 VS Code 左侧 **Run and Debug**（`Ctrl+Shift+D`），选择：

   ```text
   LaserMotion: 调试去畸变节点（Docker）
   ```

4. 按 `F5`。底部 **Debug Console** 会显示 ROS 启动输出。此时节点正在等激光数据，不命中断点是正常的。

调试开始后的操作完全可视化：

```text
F5       继续运行到下一个断点
F10      单步跳过（next，不进入函数）
F11      单步进入（step，进入自己的函数）
Shift+F11 运行到当前函数返回（finish）
Shift+F5 停止调试节点
```

停在断点后直接看左侧面板：

- **VARIABLES**：当前函数参数和局部变量；
- **WATCH**：把表达式加进去，每次停住自动更新；
- **CALL STACK**：确认自己处于回调、分段还是 ICP；
- **BREAKPOINTS**：可勾选临时禁用某个断点，避免每个 5 ms 分段都停住。

建议最先加入的 Watch 表达式：

```cpp
ranges.size()
angles.size()
start_index
beam_number
frame_base_pose.getOrigin().x()
frame_start_pose.getOrigin().x()
corrected_local_cloud->points.size()
```

`corrected_local_cloud` 仅在回调后半段的作用域内存在；当你停在函数开头时它显示不可用，属于正常的 C++ 作用域规则。

### 3.3 给 VS Code 节点准备 rosbag 数据

VS Code 的 `F5` 只启动被调试的节点，**不会**启动 `roscore`、RViz 或 rosbag。这是故意的：你可以独立控制每一帧数据何时进入断点。

在 VS Code 内置终端或第二个 `./run.sh` shell 中，启动 master：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash
roscore
```

`roscore` 启动后，**另开一个终端**执行：

```bash
# 当前 rosbag 采用 bag 时间。没有这个参数时，节点中的 ROS 时间会卡在 0。
rosparam set /use_sim_time true
```

再开一个终端，暂停播放 bag：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

cd /workspace/src/LaserMotion/bag
rosbag play --clock --pause laser.bag
```

最后开一个终端，用服务控制数据：

```bash
PAUSE_SERVICE=$(rosservice list | grep '/pause_playback$')

# 放行数据，让 VS Code 命中 scanCallback 断点后立刻再执行下一行暂停。
rosservice call "$PAUSE_SERVICE" 'data: false'
rosservice call "$PAUSE_SERVICE" 'data: true'
```

现在你可以在 VS Code 里先按 `F10` 看变量变化，最后按 `F5` 处理完当前帧；然后再在终端放行下一帧。

### 3.4 Debug Console 比较适合看的表达式

停住后，在 VS Code 底部 **Debug Console** 输入 GDB 表达式。建议不要把整个 vector 或点云一次打印出来，而是先看数量和少量元素：

```cpp
ranges.size()
ranges[0]
angles[0]
ranges[start_index + 1]
angles[start_index + 1]
frame_end_pose.getOrigin().y()
current_local_cloud.points.size()
T_previous_current_prior.getOrigin().x()
```

由于 `cppdbg` 将表达式交给 GDB，个别成员函数调用若不被 Debug Console 接受，可以将它们放到 **WATCH**，或改用终端 GDB 脚本中的 `p expression` 方式。

### 3.5 为什么不能直接 F5 `roslaunch`

`roslaunch` 是一个 Python 进程，真正要看的 `LaserUndistortion_node` 是它启动的子进程。直接调试 `roslaunch` 会进入 ROS 的 Python 启动代码，而不是你的 C++ 函数。

因此 `launch.json` 直接调试：

```text
/workspace/src/LaserMotion/LaserUndistortion_ws/devel/lib/
  LaserUndistortion/LaserUndistortion_node
```

RViz 则单独在另一个容器终端启动：

```bash
rviz -d "$(rospack find LaserUndistortion)/launch/LaserMotion.rviz"
```

这样 **VS Code 负责 C++ 单步**，**rosbag 负责数据时间**，**RViz 负责画面验证**，彼此不会抢同名节点。

---

## 4. 推荐的三终端调试流程

三终端分别做一件事，思路最清楚：

```text
终端 A：GDB + LaserUndistortion_node    → 观察 / 单步 C++
终端 B：rosbag play --pause             → 决定是否放行时间与下一帧
终端 C：rosservice / rostopic / RViz    → 查看运行状态和话题数据
```

不要在调试时使用 `roslaunch LaserUndistortion LaserUndistortion.launch` 来启动节点：launch 会直接启动同名节点，与 GDB 启动的节点冲突。GDB 模式下改为手动启动 `roscore`、GDB 节点和 rosbag。

### 3.1 终端 A：用 GDB 启动节点

先打开一个容器 shell：

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker
./run.sh
```

容器内执行：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

# 需要一个 master；如果终端 B 或 C 已启动 roscore，就不要再启动第二个。
roscore
```

另开一个**同样是终端 A 的第二个 shell**（或另开终端后再次 `./run.sh`），输入：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

cd /workspace/src/LaserMotion/LaserUndistortion_ws

gdb -q \
  -x src/LaserUndistortion/scripts/debug_laser_undistortion.gdb \
  devel/lib/LaserUndistortion/LaserUndistortion_node
```

工程提供的 `debug_laser_undistortion.gdb` 会自动设置：

```gdb
break LidarMotionUndistortion.cpp:107  # scanCallback：一帧扫描刚进入
break LidarMotionUndistortion.cpp:270  # 去畸变 TODO：一个 5 ms 分段
break laser_undistortion::IcpOdometry::process
break laser_undistortion::PclIcpEstimator::estimate
```

它只设置断点，不会自动运行节点。在 GDB 提示符输入：

```gdb
run
```

程序会先完成 ROS 初始化并等待 `/champion_scan`。此时停住是正常的：bag 还没有被放行。

### 3.2 终端 B：从暂停状态播放 bag

```bash
cd /home/zachary/workspace/ROS_NOETIC/docker
./run.sh
```

容器内执行：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

cd /workspace/src/LaserMotion/bag
rosbag play --clock --pause laser.bag
```

`--clock` 必不可少：当前 launch/节点调试使用 bag 的仿真时间，TF 查询需要 bag 的 `/clock`。`--pause` 让播放一开始就暂停。

启动后，rosbag player 会注册一个名字类似下面的节点和服务：

```text
/play_1234567890123456789
/play_1234567890123456789/pause_playback
```

数字每次启动都会变，因此不要写死完整服务名。

### 3.3 终端 C：控制 rosbag 的暂停/恢复

再开一个 shell，source 同一套环境后先找到暂停服务：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

rosservice list | grep '/pause_playback$'
```

为了每次不用复制那串动态节点名，可以在当前 shell 设置：

```bash
PAUSE_SERVICE=$(rosservice list | grep '/pause_playback$')
printf '%s\n' "$PAUSE_SERVICE"
```

恢复播放：

```bash
rosservice call "$PAUSE_SERVICE" 'data: false'
```

再次暂停：

```bash
rosservice call "$PAUSE_SERVICE" 'data: true'
```

注意 `data: false` 的含义是 **paused = false，所以恢复**；`data: true` 才是暂停。

### 3.4 最小操作节奏

第一次建议这样做：

1. GDB 里输入 `run`，让节点等待数据；
2. 终端 B 启动 `rosbag play --clock --pause laser.bag`；
3. 终端 C 执行一次 `rosservice call "$PAUSE_SERVICE" 'data: false'`；
4. GDB 命中 `scanCallback` 后，**立刻**在终端 C 执行 `data: true`；
5. 在 GDB 中观察这一帧，最后输入 `continue` 让此帧回调结束；
6. 再恢复一次 bag，进入下一帧。

`data: false` 到 `data: true` 之间可能已经发出多条 `/tf` 和 `/odom`，这是正常且必要的；关键是让 `/champion_scan` 尽量只经过一帧。即便多进入一帧，GDB 的 `scanCallback` 断点也会先拦住它，数据不会悄悄跑完。

---

## 4. 精确到“每一帧扫描”暂停（可选）

比“恢复后手动暂停”更稳定的方法是让 rosbag 每发布一帧 `/champion_scan` 就自动暂停。

```bash
cd /workspace/src/LaserMotion/bag
rosbag play --clock \
  --pause-topics /champion_scan \
  --bags laser.bag
```

这个模式下，player 会在每一条 `/champion_scan` 后自动进入暂停状态。由于 `/tf` 与 `/odom` 也会同时到达，节点查变换时需要注意：若断点正停在需要未来 TF 的位置，先恢复 rosbag 补充 TF，再回到 GDB 继续单步即可。

建议优先用第 3 节的 `--pause` 做第一次实验。完全理解时间流、TF 缓冲和回调关系后，再切换到 `--pause-topics`。

---

## 5. GDB 最常用的十个命令

| 命令 | 含义 | 在本作业中的典型用法 |
|---|---|---|
| `bt` | 当前调用栈 | 确认自己是在回调、分段函数还是 ICP 里 |
| `list` | 显示当前位置附近源码 | 停住后先看将执行哪一行 |
| `next` / `n` | 执行下一行，不进入函数 | 看循环和赋值，避免钻进 TF/PCL 库 |
| `step` / `s` | 执行下一行，进入函数 | 想进入自己的 `buildCloudInLaser()` 时使用 |
| `finish` | 执行到当前函数返回 | 已看懂一个分段后，回到 `Lidar_Calibration()` |
| `continue` / `c` | 运行到下一个断点 | 放当前帧走到下一个阶段 |
| `info locals` | 打印当前局部变量 | 优先查看 `beam_count`、时间和点云大小 |
| `p expression` | 求值并打印表达式 | 例如 `p ranges.size()` |
| `display expression` | 每次停住都自动显示 | 跟踪某个固定变量，如 `display beam_number` |
| `undisplay N` | 取消自动显示 | 变量太多导致输出拥挤时使用 |

STL 容器的实际写法：

```gdb
# vector<double> 的元素数和前几个元素
p ranges.size()
p angles.size()
p ranges[0]
p angles[0]
p ranges[10]
p angles[10]

# 点云点数，及第一点坐标
p corrected_local_cloud->points.size()
p corrected_local_cloud->points[0].x
p corrected_local_cloud->points[0].y

# tf 的平移分量
p frame_start_pose.getOrigin().x()
p frame_start_pose.getOrigin().y()
p frame_end_pose.getOrigin().x()
p frame_end_pose.getOrigin().y()

# 可长期显示。显示编号通常是 1；以实际输出为准。
display beam_number
display start_index
```

遇到很多模板内部字段时，不要先试图读完整个对象。先从“大小、一个下标、一个坐标分量”开始。这比直接 `p ranges` 或 `p *corrected_local_cloud` 清晰得多。

---

## 6. 第一次要逐行看的：去畸变 TODO

在 `LidarMotionUndistortion.cpp:270` 设置的断点命中后，GDB 停在：

```cpp
void Lidar_MotionCalibration(
    tf::Stamped<tf::Pose> frame_base_pose,
    tf::Stamped<tf::Pose> frame_start_pose,
    tf::Stamped<tf::Pose> frame_end_pose,
    std::vector<double>& ranges,
    std::vector<double>& angles,
    int start_index,
    int& beam_number)
```

这里每次停住不是一整帧，而是一个不重叠的 **5 ms 分段**。这正好适合建立“整帧参考系”与“分段瞬时位姿”之间的区别。

### 6.1 第一次停住，先回答四个问题

```gdb
info args
p start_index
p beam_number
p ranges.size()
p angles.size()

p frame_base_pose.getOrigin().x()
p frame_base_pose.getOrigin().y()
p frame_start_pose.getOrigin().x()
p frame_start_pose.getOrigin().y()
p frame_end_pose.getOrigin().x()
p frame_end_pose.getOrigin().y()
```

请在纸上（或笔记里）写下：

1. `start_index` 是这段第几束，而非第几帧；
2. `beam_number` 是这个 5 ms 段的束数；
3. `frame_base_pose` 是整帧第 0 束的位姿，在所有分段中应该保持不变；
4. `frame_start_pose`、`frame_end_pose` 只描述当前小段在 `odom` 内如何运动。

可以用下面不变量验证“整帧基准没有被错改”为分段基准：

```gdb
# 记住第一次分段的 frame_base_pose.x；连续 c 到下一分段后再次查看。
p frame_base_pose.getOrigin().x()
p frame_start_pose.getOrigin().x()
```

下一个分段中 `frame_start_pose` 通常变化；`frame_base_pose` 应仍对应整帧开头。

### 6.2 给 TODO 写完实现后，重点观察一束点

去畸变目标公式是：

$$
p_{base} = T_{odom \leftarrow laser}(t_0)^{-1}
           T_{odom \leftarrow laser}(t_j)
p_{laser}
$$

不要一开始单步整段所有束。选择一个 `j`，只观察它的前后值。

```gdb
# 假设当前段至少有 2 束，先观察段内第 1 束。
p start_index
p ranges[start_index + 1]
p angles[start_index + 1]
```

再用 `next` 走到 TODO 中这束完成回写的位置，重新打印：

```gdb
p ranges[start_index + 1]
p angles[start_index + 1]
```

正确理解不等于“range 一定变化很大”：

- 机器人静止时，数值应几乎不变；
- 匀速平移时，距离和角度可小幅变化；
- 有旋转时，`angle` 的变化通常更直观；
- 结果变多少取决于该束的距离、方向与机器人轨迹。

最重要的是变换方向：先把瞬时点放到 `odom`，再用**整帧初始位姿的逆**拉回本帧起始激光坐标系。不要把 `frame_start_pose.inverse()` 当作所有分段的基准；它会让每一段落在不同参考系。

### 6.3 观察插值端点的两个不变量

写完 TODO 后，用条件断点只停在段首/段尾的迭代。代码里 `j` 是局部变量，因此要先 `next` 进入 `for` 循环后再设置条件断点：

```gdb
# 在循环第一轮停住后：
p j
p alpha

# 只在最后一束停住（先确认 beam_number 的值）
break LidarMotionUndistortion.cpp:161 if j == beam_number - 1
continue
```

预期：

```text
j == 0                    → alpha == 0 → 使用 frame_start_pose
j == beam_number - 1      → alpha == 1 → 使用 frame_end_pose
beam_number == 1          → alpha == 0 → 不发生除零
```

若调试后新设的行号有偏差，使用 `list` 找到 `alpha` 那一行，再改用显示出的实际行号。源码修改后，每次都要重新 `catkin_make -DCMAKE_BUILD_TYPE=Debug`，否则 GDB 行号与正在跑的可执行程序不一致。

---

## 7. 第二次看：点如何从极坐标变成点云

去畸变函数返回后，回调在 `LidarMotionUndistortion.cpp:209` 构建局部点云：

```cpp
const PointCloudPtr corrected_local_cloud =
    buildCloudInLaser(corrected_ranges, corrected_angles);
```

在该行附近设断点：

```gdb
break LidarMotionUndistortion.cpp:209
continue
```

进入后执行：

```gdb
next                  # 先执行完 buildCloudInLaser
p corrected_local_cloud->points.size()
p corrected_local_cloud->points[0].x
p corrected_local_cloud->points[0].y
```

你应该把它与极坐标公式连起来：

$$
x = r\cos\theta, \qquad y = r\sin\theta
$$

同一个索引的 `corrected_ranges[i]` 和 `corrected_angles[i]`，会在 `buildCloudInLaser()` 中变为一个二维点。因为无效 `range` 会被跳过，点云的第 `k` 点**未必**对应数组下标 `k`，调试时不要把这两者错误地一一对应。

接着在回调的 `LidarMotionUndistortion.cpp:212` 处，局部点云将以扫描起始位姿放进 `odom`：

```cpp
publishCloudInOdom(*corrected_local_cloud, asTransform(frame_start_pose),
                   start_time, corrected_cloud_pub_);
```

这就是为什么 `/laser_motion/corrected_local_cloud` 的 header 是 `base_laser`，而绿色 `/laser_motion/corrected_cloud` 的 header 是 `odom`。

---

## 8. 第三次看：ICP 外框如何处理两帧数据

断点 `icp_odometry.cpp:168` 对应：

```cpp
IcpProcessResult IcpOdometry::process(
    const PointCloud& current_local_cloud,
    const tf::Transform& T_odom_laser_current,
    const ros::Time& current_stamp)
```

这里输入已是**去畸变后的当前帧局部点云**。最初会看到：

```gdb
p current_local_cloud.points.size()
p current_stamp.toSec()
```

第一帧时，`previous_scan_` 为空，代码只缓存当前帧：

```text
first_scan_cached
```

第二个合格帧才会构造 odom 给出的初值：

```cpp
T_previous_current_prior =
    previous_scan_->T_odom_laser.inverse() * T_odom_laser_current;
```

它把“当前帧局部坐标”映射到“上一帧局部坐标”。GDB 里可查看：

```gdb
# 断在 icp_odometry.cpp:214 后
p T_previous_current_prior.getOrigin().x()
p T_previous_current_prior.getOrigin().y()
p T_previous_current_prior.getOrigin().z()
```

需要把旋转看成 yaw 时，可让 GDB 调用 TF 工具函数：

```gdb
p tf::getYaw(T_previous_current_prior.getRotation())
```

然后在 `icp_odometry.cpp:227` 进入 `PclIcpEstimator::estimate()`：

```gdb
step
info args
p source_current.points.size()
p target_previous.points.size()
```

请特别确认方向：

```text
source_current = 当前帧
 target_previous = 上一帧
T_previous_current = 当前帧 -> 上一帧
```

当前 `PclIcpEstimator::estimate()` 会明确返回：

```text
ICP solver TODO: implement PclIcpEstimator::estimate
```

因此调用返回后，状态是 `estimator_not_implemented`，`/laser_motion/icp/current_aligned` 不发布。这是设计上的预期，而不是调试失败。`previous_cloud`（蓝）与 `current_before`（橙）仍可在 RViz 里显示，它们用于检查 odom 初值的合理性。

---

## 9. 用 RViz 和命令行验证你在 GDB 里看到的东西

GDB 不是替代 RViz；两者观察的是同一条管线的不同层。

### 9.1 单独打开 RViz

GDB 调试模式中不使用 launch，因此可以在第三或第四个终端手动打开 RViz：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash

rviz -d "$(rospack find LaserUndistortion)/launch/LaserMotion.rviz"
```

Fixed Frame 保持为 `odom`。默认 ICP 诊断项为关闭状态，请在 RViz 左栏勾选：

- `ICP Previous Cloud (blue)`；
- `ICP Current Before (orange)`；
- ICP 实现并通过质量门控后才会出现 `ICP Current Aligned (cyan)`。

### 9.2 只打印一个点云头，不把整片点云刷满终端

```bash
rostopic echo -n 1 /laser_motion/corrected_local_cloud/header
rostopic echo -n 1 /laser_motion/corrected_cloud/header
rostopic echo -n 1 /laser_motion/icp/previous_cloud/header
```

你应该看到：

```text
corrected_local_cloud  → frame_id: base_laser
corrected_cloud        → frame_id: odom
icp/previous_cloud     → frame_id: odom
```

只检查一个消息的字段范围时，ROS1 命令行也可以使用路径过滤：

```bash
rostopic echo -n 1 /champion_scan/ranges
rostopic echo -n 1 /champion_scan/angles
```

完整 `ranges` 数组可能很长。结合 GDB 的 `p ranges[0]`、`p ranges[10]` 会更容易理解。

### 9.3 验证 TF 与输入话题是否齐全

```bash
rostopic hz /champion_scan
rosrun tf tf_echo /odom /base_laser
rosnode info /LaserUndistortion_Node
```

在 bag 被暂停时，这些更新会停止或变慢，是预期现象；恢复播放后才继续变化。

---

## 10. 遇到这些现象时，不要误判

### 10.1 GDB 断点从不命中

依次检查：

```bash
# 节点是否真的由 GDB 启动？应只存在一个同名节点。
rosnode list

# 当前运行的二进制是否来自 Debug workspace？
rosnode info /LaserUndistortion_Node

# 消息是否到达？
rostopic hz /champion_scan
```

常见原因：

- 用 `roslaunch` 又启动了一个同名节点，或 GDB 中的节点未 `run`；
- bag 仍处于 `--pause` 状态；
- `rosbag` 已播完；重新启动 `rosbag` 并从暂停开始；
- 改过源码却没有以 Debug 模式重新编译；
- 没有 source `champion_nav_msgs/devel/setup.bash`，节点无法正确加载自定义消息包。

### 10.2 停在 `getLaserPose()`，等待 TF 或报超时

这说明没有足够的 `/tf` 数据到达该扫描时间。确认：

- bag 使用 `--clock`；
- 暂停过早时，先从终端 C 恢复 player，让 `/tf` 与 `/odom` 继续填入缓存；
- `use_sim_time` 为 true 时，不能指望真实墙钟时间替代 bag 的时间；
- 不要同时开两个 `roscore` 或两个 bag player。

### 10.3 红绿点云重合

如果 `Lidar_MotionCalibration()` 仍是 TODO，这是完全正确的预期。它目前不改写 `ranges` 和 `angles`，所以 `corrected_ranges == ranges`。

完成 TODO 后，静止片段中红绿依然可以重合；要在机器人运动过程中、对着墙面或直线结构比较，绿色应更集中。

### 10.4 第二帧后 ICP 状态仍是 `estimator_not_implemented`

也是预期。`PclIcpEstimator::estimate()` 目前只返回 `kNotImplemented`。框架已经会给你准备：

- 有限点过滤与 VoxelGrid 降采样；
- 当前帧 / 上一帧缓存；
- odom 初值；
- 蓝色 target 与橙色 initial-guess 诊断云；
- fitness、平面约束与 odom 偏差质量门控。

但真正的 PCL ICP 调用仍要在 `icp_odometry.cpp:136` 手写。

### 10.5 GDB 显示 `<optimized out>`、单步跳来跳去

这几乎总是仍在跑 Release 二进制，或用了源码与二进制不一致的旧编译产物。重新执行第 2 节的 Debug 构建，然后重新启动 GDB 节点。

### 10.6 `rosbag` 或 `roslaunch` 报 duplicate package

当前 `LaserUndistortion_ws` 同时有 `src/LaserUndistortion` 和 `install/share/LaserUndistortion`，若在仓库根目录直接 source root workspace，ROS 可能报同名包冲突。

对于本指南的调试流程，始终只 source：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash
```

不要在这个流程中 `source /workspace/devel/setup.bash`。如果环境已经污染，开一个新的 `./run.sh` shell 后按上述顺序 source 即可。

---

## 11. 最值得做的三轮小实验

### 实验 A：验证第一帧只会缓存，不能 ICP

1. 用 `--pause` 启动 bag；
2. 放行一帧；
3. GDB 在 `IcpOdometry::process()` 查看 `previous_scan_`；
4. `continue` 完成回调；
5. 终端输出不应有 ICP 匹配，而是内部缓存第一帧；
6. 放行第二帧，才会走到 `PclIcpEstimator::estimate()`。

理解结果：ICP 至少需要两组点云；“当前帧”自身不能与不存在的上一帧匹配。

### 实验 B：验证整帧基准与分段基准不同

1. 在 `Lidar_MotionCalibration()` 首次命中时记录 `frame_base_pose` 与 `frame_start_pose` 的 `x/y`；
2. `continue` 到第二个 5 ms 分段；
3. 再次打印二者；
4. 检查 `frame_start_pose` 变化，而 `frame_base_pose` 仍是整帧第 0 束的基准。

理解结果：逐束补偿的共同目标是本帧首束坐标系，不是每段自己的首束坐标系。

### 实验 C：在 RViz 里验证 ICP 初值方向

1. 放行至少两帧；
2. 在 RViz 勾选蓝色 `previous_cloud` 和橙色 `current_before`；
3. 在 `icp_odometry.cpp:214` 观察 `T_previous_current_prior` 的平移与 yaw；
4. 对应源码 `previous_pose.inverse() * current_pose`，确认它是 **current → previous**；
5. 思考：如果把乘法顺序倒过来，橙色云应该往什么错误方向偏？

理解结果：ICP 的 source/target 方向与变换方向是同一件事的两种表述。方向一旦反了，数值即便收敛也不会代表正确的相对位姿。

---

## 12. 调试结束后的清理

按这个顺序停止即可：

1. 终端 B 对 `rosbag play` 按 `Ctrl-C`；
2. GDB 输入 `quit`（若仍在运行，确认结束 inferior）；
3. 运行 `roscore` 的终端按 `Ctrl-C`；
4. 关闭 RViz。

Docker 容器本身是常驻的，退出 `./run.sh` shell 不会删除容器，也不会丢失源码与 Debug 构建产物。

---

## 13. 一张随手可查的调试卡

```bash
# 构建（只需源码修改后执行）
cd /home/zachary/workspace/ROS_NOETIC/docker
./run.sh "source /opt/ros/noetic/setup.bash && \
  source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash && \
  cd /workspace/src/LaserMotion/LaserUndistortion_ws && \
  catkin_make -DCMAKE_BUILD_TYPE=Debug"

# 终端 A：先启动 roscore，再启动 GDB
source /opt/ros/noetic/setup.bash
source /workspace/src/LaserMotion/champion_nav_msgs/devel/setup.bash
source /workspace/src/LaserMotion/LaserUndistortion_ws/devel/setup.bash
roscore

# 在另一个终端（roscore 已启动后）运行一次：
rosparam set /use_sim_time true

gdb -q -x /workspace/src/LaserMotion/LaserUndistortion_ws/src/LaserUndistortion/scripts/debug_laser_undistortion.gdb \
  /workspace/src/LaserMotion/LaserUndistortion_ws/devel/lib/LaserUndistortion/LaserUndistortion_node
# GDB 内：run

# 终端 B：暂停播放 bag
cd /workspace/src/LaserMotion/bag
rosbag play --clock --pause laser.bag

# 终端 C：恢复 / 暂停
PAUSE_SERVICE=$(rosservice list | grep '/pause_playback$')
rosservice call "$PAUSE_SERVICE" 'data: false'  # 恢复
rosservice call "$PAUSE_SERVICE" 'data: true'   # 暂停

# GDB 内的第一组观察命令
bt
info locals
p ranges.size()
p ranges[0]
p angles[0]
p frame_base_pose.getOrigin().x()
p frame_start_pose.getOrigin().x()
next
```

真正要记住的不是所有 GDB 命令，而是这条数据契约：

```text
一帧输入 ranges/angles
  → 每束在瞬时 laser 坐标系中被测量
  → 统一变到“本帧第一束 laser 坐标系”
  → 当前帧局部点云交给 ICP
  → current → previous 的相对位姿由 ICP 求解
  → 仅为展示时才变到 odom
```
