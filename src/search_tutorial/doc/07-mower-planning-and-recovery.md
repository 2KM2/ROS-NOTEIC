# 割草机路径规划、避障与脱困如何组合

割草机需要“覆盖规划 + 全局路径规划 + 局部避障 + 脱困状态机”协同工作。A*、D* Lite、JPS 只解决点到点路径，不负责整块草坪覆盖，也不负责从物理卡滞中脱离。

## 总体架构

```text
覆盖任务规划 → 全局路径规划 → 局部控制器 → 安全监督器
       ↑                                      ↓
       └──────── 恢复管理器 / 任务状态机 ────────┘
```

| 模块 | 负责内容 | 常用方法 |
|---|---|---|
| 覆盖规划器 | 草坪分区、割草线和访问顺序 | BCD、弓字形、STC |
| 全局规划器 | 当前点到割草线或充电桩的路径 | A*、D* Lite、Hybrid A* |
| 局部控制器 | 路径跟踪和小范围绕行 | Pure Pursuit、TEB、DWA、MPC |
| 安全监督器 | 碰撞、抬起、倾斜、堵转、人员接近 | 独立安全链路 |
| 恢复管理器 | 后退、旋转、重试、临时禁区、重规划 | 状态机或行为树 |

## 1. 正常割草

覆盖规划器先把区域分解为平行割草线。每条割草线分为“转场”和“割草”两个阶段：

```text
选择下一条未完成割草线
        ↓
全局规划到割草线入口
        ↓
局部控制器跟踪割草线并开启刀盘
        ↓
记录已覆盖区间，选择下一条线
```

转场可以使用更自由的路径；正式割草时应尽量贴合覆盖轨迹，以降低漏割率和重复率。

## 2. 遇到障碍

```text
检测到障碍
    ↓
是否涉及人员、宠物、抬起或碰撞？
  ├─ 是：停刀并停车，进入安全等待
  └─ 否：更新局部 costmap
              ↓
        局部控制器能否绕过？
          ├─ 能：短距离绕行后回归原路径
          └─ 不能：更新全局地图并重新规划
```

小型临时障碍可以由局部控制器绕过；完全截断当前路径时，写入 costmap 并调用 A* 或 D* Lite。
D* Lite 负责重新计算“去哪”，不负责解决车轮已经打滑或底盘被卡住的问题。

```text
传感器发现新障碍
    → costmap 更新
    → dstar.updateCell(x, y, true)
    → 清空局部控制器旧轨迹
    → 跟踪新路径
```

如果障碍把整个区域切成多个连通分量，需要重新排序剩余割草线，并把跳过部分生成补割任务。

## 3. 被困检测

被困表示控制命令没有产生预期运动，建议融合：

- 下发非零速度但融合定位位移很小；
- 编码器显示轮子转动、地图坐标没有变化，说明可能打滑；
- 电机电流持续过高，说明可能堵转；
- IMU 横滚或俯仰过大，说明可能卡在坑或坡坎；
- 同一碰撞开关连续触发；
- 轨迹在小范围内重复，说明局部避障循环。

基础判据可以写成：

```text
commanded_distance > 0.5 m
and actual_progress < 0.1 m
and duration > 3 s
```

实际位移应来自融合定位或外部定位，不能只依赖轮速里程计，否则车轮空转会被误判为正常前进。

## 4. 脱困状态机

```text
NORMAL
  ↓ 无法前进
STOP_AND_CHECK
  ↓
BACKUP → ROTATE → RETRY
  ├─ 成功 ─────────→ GLOBAL_REPLAN → NORMAL
  ├─ 再次失败 ─────→ 增大动作幅度
  └─ 严重倾斜/抬起 → EMERGENCY_STOP

超过最大尝试次数 → 临时禁区 → 停刀停车 → WAIT_FOR_HELP
```

恢复动作不能每次完全相同，否则容易在墙角或 U 形障碍中循环。可逐次升级：

| 尝试 | 后退距离 | 转向角度 | 方向选择 |
|---|---:|---:|---|
| 1 | 0.3 m | 30° | 传感器更空的一侧 |
| 2 | 0.5 m | 60° | 与上一次相反 |
| 3 | 0.8 m | 90° | 最大空闲扇区 |
| 4 | - | - | 扩大临时禁区并全局重规划 |

脱困成功后不能直接恢复旧速度命令，应更新定位、清空旧轨迹、加入临时禁区，再规划到当前割草线的安全回归点。

## 5. 覆盖进度与补割

为每条割草线保存状态和已完成区间：

```cpp
enum class SwathState { kPending, kActive, kPartiallyDone, kCompleted, kBlocked };
struct CoveredInterval { double begin_s; double end_s; };
```

绕障或脱困后，原割草线可能留下一个未覆盖 gap。任务结束前，将覆盖栅格与目标区域做差集，只对可达 gap 生成补割轨迹。至少记录覆盖率、重复覆盖率、漏割面积、总路径长度、转弯次数、重规划次数和脱困次数。

## 6. 推荐状态机

```text
IDLE → PLAN_COVERAGE → SELECT_NEXT_SWATH → NAVIGATE_TO_SWATH → MOW_SWATH
                                      │             │
                                      │             ├─ 小型障碍 → LOCAL_AVOID
                                      │             ├─ 路径阻断 → GLOBAL_REPLAN
                                      │             ├─ 进度不足 → RECOVERY
                                      │             └─ 安全事件 → EMERGENCY_STOP
                                      └─ 正常完成 → SELECT_NEXT_SWATH

LOCAL_AVOID：成功 → MOW_SWATH；失败 → GLOBAL_REPLAN
RECOVERY：成功 → GLOBAL_REPLAN；多次失败 → WAIT_FOR_HELP
```

## 7. ROS 节点划分

```text
mower_mission_manager  管理割草区域、割草线、回充和任务状态机
coverage_planner       BCD / 弓字形覆盖，输出割草线与补割任务
global_planner         A* / D* Lite / Hybrid A*，输出 nav_msgs/Path
local_controller       Pure Pursuit / TEB / MPC，输出底盘速度
recovery_manager       stuck 检测、恢复动作、临时禁区、重规划请求
safety_controller      急停、停刀、碰撞和人员保护
```

建议接口包括 `/mower/global_path`、`/mower/coverage_grid`、`/mower/temporary_obstacles`、`/mower/recovery_event` 和独立的 `/mower/safety_stop`。

## 8. 安全优先级

```text
人员接近 / 抬起 / 翻倒 > 刀盘停止 > 底盘急停 > 碰撞避障
> 脱困 > 路径重规划 > 路径跟踪 > 覆盖效率
```

急停和停刀不能依赖路径规划器或普通 ROS 话题正常运行，实际产品应使用独立硬件链路或安全控制器。

## 9. 算法组合建议

```text
BCD + 弓字形覆盖       生成完整割草任务
D* Lite                地图局部变化后的增量全局重规划
Hybrid A*              严格考虑最小转弯半径的转场
Pure Pursuit / MPC      路径跟踪
局部 costmap           表达动态障碍
恢复状态机             后退、转向、重试和失败升级
覆盖栅格               保存进度并生成补割任务
独立安全控制器         急停、停刀和人员保护
```
