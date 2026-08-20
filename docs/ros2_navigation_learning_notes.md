# ROS 2 导航学习与面试复习笔记

> 记录日期：2026-08-19。本文来自 PredictiveNav2 静态导航 baseline 的实际运行与调试，不是脱离项目的概念摘抄。

## 1. 为什么要看 TF 链？

### 问题

什么是 TF 链？为什么机器人导航需要它？

### 回答

TF（transform）记录不同坐标系之间的位置和朝向关系。机器人系统中的传感器、车体、里程计和地图各自有坐标系；必须通过 TF 才能把一份数据换算到另一个参考系。

本项目的核心链：

```text
map → odom → base_footprint → base_link → top_deck_link → lidar_link
```

- `map`：全局静态地图坐标系；目标点和全局路径通常在这里表达。
- `odom`：连续平滑的里程计坐标系；短期运动可靠，但会积累误差。
- `base_footprint`：地面上的机器人中心，是导航关心的机器人位置。
- `base_link`：车体坐标系；轮子、IMU、LiDAR 等结构都从这里挂出。
- `lidar_link`：2D LiDAR 测量距离时使用的参考坐标系。

在本项目中：

```text
AMCL            发布 map → odom
Gazebo 里程计    发布 odom → base_footprint
URDF / robot_state_publisher
                发布车体、传感器和关节的内部 TF
```

## 2. 激光数据的 `header` 是什么？

### 问题

执行下面命令后，输出的 `stamp` 和 `frame_id` 是什么意思？

```bash
ros2 topic echo --once /scan --field header
```

### 回答

典型输出：

```text
stamp:
  sec: 8460
  nanosec: 700000000
frame_id: lidar_link
```

- `stamp`：这帧激光采集的时刻。本项目使用 Gazebo 仿真时间，不是电脑墙钟时间。
- `frame_id: lidar_link`：这帧激光中的距离，是从 LiDAR 自身为原点测出来的。

ROS 会根据“数据时间戳 + 数据坐标系”在 TF 缓存中查找变换，将激光逐级转换到地图：

```text
lidar_link → … → base_footprint → odom → map
```

若时间戳、`frame_id` 或 TF 链不正确，激光可能无法显示、报 TF 错误，或在 RViz 地图中错位。

## 3. 如何阅读 `ros2 topic info /scan -v`？

### 问题

终端输出很长，哪些内容最重要？

### 回答

只先看三项：

```text
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 6
```

它们的含义是：

- `LaserScan`：数据是一圈二维激光距离。
- 一个发布者：本项目的 `dynamic_navigation_bridge` 将 Gazebo 中的雷达数据桥接为 ROS `/scan`。
- 多个订阅者：例如 AMCL、局部/全局 costmap、`scan_safety_guard` 都使用激光。

当前阶段可以忽略 `GID`、type hash 和大多数 QoS 细节；出现传感器数据收不到时再专门学习 QoS。

## 4. 为什么点错 2D Pose Estimate 后激光会错位？

### 问题

为什么手动点了 `2D Pose Estimate` 后，RViz 中的红色激光会“乱掉”？

### 回答

`2D Pose Estimate` 会向 `/initialpose` 发布一个新的 AMCL 初始位置和朝向。AMCL 会据此调整 `map → odom`。

本项目的仿真出生位置已由 bringup 自动设置；若手动输入的位置或朝向不符合 Gazebo 中的真实小车位置，`map → odom` 就会错误。激光本身没有坏，但它转换到 `map` 后会被放到错误位置。

因此静态 baseline 的正确做法是：不手动使用 `2D Pose Estimate`，让启动脚本完成初始定位。

## 5. 一个 RViz 目标怎样让小车移动？

```text
RViz Nav2 Goal
→ Navigation 2 面板
→ /navigate_to_pose action
→ 全局规划器生成 /plan
→ DWB 局部控制器生成 /cmd_vel
→ scan_safety_guard
→ /cmd_vel_safe
→ Gazebo DiffDrive
→ odom / TF / scan
→ AMCL 与 Nav2 继续闭环
```

注意：Jazzy 的 `Nav2 Goal` 工具本身只发内部 RViz 信号；必须有 `Navigation 2` 面板，才能真正发送 `NavigateToPose` 动作。

## 6. 看速度话题时应该怎么看？

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
```

- `/cmd_vel`：Nav2 希望机器人执行的速度。
- `/cmd_vel_safe`：激光安全过滤后，实际交给 Gazebo DiffDrive 的速度。

机器人空闲时，这两个话题不一定持续有数据；发送 Nav2 目标并开始移动后才会看到速度消息。

## 7. 这些知识容易忘吗？重要吗？

会忘命令和参数名，这是正常的；不需要死记。

应保留的判断框架是：

```text
数据从哪个 frame 来？
数据是什么时间产生的？
怎样经 TF 变换到 map？
哪个节点发布，哪些节点订阅？
```

这对 PredictiveNav2 很重要：未来动态障碍物检测从 `/scan` 开始，障碍物位置必须转换到稳定参考系后，才能跟踪、预测并送进 MPPI 风险评价。

## 8. 面试可能怎样问？

面向 ROS、机器人导航或移动机器人岗位时，以下问题很常见：

1. `map`、`odom`、`base_link` / `base_footprint` 分别是什么？
2. 为什么 AMCL/SLAM 发布 `map → odom`，而里程计发布 `odom → base_footprint`？
3. LiDAR 消息中的 `frame_id` 和时间戳为什么重要？
4. RViz 中激光和地图错位时，如何排查？
5. 为什么 TF 树不能有环，且每个子 frame 只能有一个父 frame？
6. 一个导航目标如何变成底盘速度命令？
7. `/cmd_vel` 与 `/cmd_vel_safe` 为什么分开？

对这些问题，最好的回答方式不是背定义，而是结合本项目的真实链路和调试经历说明。

## 9. 面试示范回答：Gazebo 激光与 AMCL 定位

### 问题

Gazebo 是怎么发布激光的？AMCL 又如何通过激光和静态地图，让机器人知道自己在地图中的位置？`

### 一分钟回答

> 在我的项目里，Gazebo 在 `lidar_link` 上挂载了一个 `gpu_lidar` 传感器。它按设定频率向四周发射虚拟激光束，与仿真世界的碰撞几何体求交，得到每个角度上的距离。Gazebo 先发布 `gz.msgs.LaserScan`，再由 `ros_gz_bridge` 转换成 ROS 的 `/scan`，消息类型是 `sensor_msgs/msg/LaserScan`，并带有 `frame_id=lidar_link` 和仿真时间戳。
>
> AMCL 同时使用静态 `/map`、`/scan` 和 `/odom`，采用粒子滤波。它先根据里程计增量预测每个粒子的运动，再比较“如果机器人处于该粒子位置，地图中预期看到的激光”和实际 `/scan` 是否一致；匹配好的粒子权重更高，之后重采样，得到机器人在 `map` 中的位姿估计。
>
> 里程计持续发布 `odom → base_footprint`，AMCL 不直接替代它，而是发布 `map → odom` 的校正变换，使 `map → odom → base_footprint` 等于 AMCL 的地图位姿估计。这样 odom 保持局部连续和平滑，地图定位又能纠正长期漂移，Nav2 就能知道机器人在地图中的位置并规划路径。

### 追问：为什么需要初始位姿？

> 仿真中小车出生点已知，所以我通过 `/initialpose` 在 AMCL 启动后提供出生位置附近的初始粒子分布；否则 AMCL 需要在整张地图中搜索，收敛会更慢且更不稳定。

### 追问：项目中的具体设置是什么？

> 项目的 LiDAR 为 10 Hz、720 束、360°、量程 0.08 到 12 米，并加入 0.01 米标准差的高斯噪声；Gazebo 里程计约以 50 Hz 发布。

## 10. 自检清单

能用自己的话解释以下内容，就算真正掌握了本阶段：

- [ ] 能画出 `map → odom → base_footprint → lidar_link`。
- [ ] 能说明 `/scan.header.frame_id` 和 `stamp` 的作用。
- [ ] 能说明 AMCL 为什么会影响激光在 RViz 中的位置。
- [ ] 能说明 Nav2 Goal、路径、DWB、速度安全过滤和 Gazebo 的顺序。
- [ ] 能在激光错位或机器人不动时列出基本排查顺序。

## 11. 一次 Nav2 Goal 怎样真正驱动车轮？

一次导航不是 RViz 直接控制轮子，而是下列闭环：

```text
RViz 的 Nav2 Goal
  → /navigate_to_pose Action
  → bt_navigator（组织任务）
  → planner_server（计算 /plan 全局路径）
  → controller_server 内的 DWB（持续选择 /cmd_vel）
  → scan_safety_guard（激光最终安全检查）
  → /cmd_vel_safe
  → Gazebo DiffDrive（计算左右轮速度并模拟运动）
  → /odom、TF、/scan 反馈给 AMCL 与 Nav2，进入下一轮控制
```

### RViz：把人的意图变成导航目标

在地图中点击的位置是目标 `x, y`，拖出的方向是目标朝向 `yaw`。RViz 将它作为 `map` 坐标系下的 `NavigateToPose` Action 目标发送给 `/navigate_to_pose`。Action 适合这种耗时任务：它有目标、持续反馈、取消和最终成功/失败结果。

本项目的 RViz 还需要 `nav2_rviz_plugins/Navigation 2` 面板：`Nav2 Goal` 工具负责取得鼠标给出的位姿，该面板才会创建真正的 Action 客户端并发送请求。

### bt_navigator：任务总指挥，不直接开车

`bt_navigator` 是行为树调度器。它接到目标后，依次请求规划、跟踪，并在失败时协调旋转、后退、等待等恢复行为。它不直接找路，也不直接发布轮速；可以把它理解为“导航流程的项目经理”。

### planner_server：产生应该走哪些点的 `/plan`

规划器知道当前 `map` 位姿、目标位姿以及全局代价地图，输出一串从起点到终点的 `PoseStamped` 点，构成 `/plan`。路径说明“总体应经过哪里”，但不包含此刻轮子该转多快。

### controller_server / DWB：每 1/20 秒决定此刻的速度

本项目的 `controller_frequency` 是 `20.0 Hz`。DWB 每次读取当前位姿、全局路径与局部代价地图，尝试多组“线速度 + 角速度”，短时间模拟它们会把车带到哪里，排除碰撞候选，再按路径贴合、靠近目标、朝向目标、避障和防振荡等标准打分。最低总代价的候选成为下一条 `/cmd_vel`。因此 `/cmd_vel` 只是短暂速度指令，而不是“直接走到终点”的一次性命令。

### scan_safety_guard：独立的最终安全闸门

`scan_safety_guard` 同时订阅 `/scan` 和 `/cmd_vel`。若前方实测激光距离过近，它会阻止或限制向前的速度；通过检查后才发布 `/cmd_vel_safe`。这层不替代 DWB 的局部避障，而是在异常、调参不当或局部地图短暂滞后时提供最后一道保护。

### Gazebo DiffDrive 与反馈闭环

桥接后的安全速度由 Gazebo 的差速驱动插件消费。插件将线速度、角速度换成左右轮速度，物理引擎模拟机器人运动；与此同时 Gazebo 又持续产生 `/odom`、TF 和 `/scan`。AMCL 更新 `map → odom`，DWB 依据最新状态再选一次速度，于是形成持续闭环。

## 12. 项目中的全局规划器：Navfn + A* 风格

`nav2_dwb.yaml` 中的实际配置是：

```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: nav2_navfn_planner::NavfnPlanner
    tolerance: 0.25
    use_astar: true
    allow_unknown: true
```

这里的名称要分开看：

- `planner_server`：提供规划服务的 Nav2 节点。
- `GridBased`：你为这个规划器起的插件实例名，不代表一种新算法。
- `NavfnPlanner`：真正加载的 Nav2 全局规划器实现。
- `use_astar: true`：让它以 A* 搜索方式在二维栅格代价地图上找路径。

地图会被表达成许多 0.05 m × 0.05 m 的格子。墙和膨胀区的格子代价高或不可通行，可通行的格子代价低。A* 从起点开始扩展候选格子，对每一个候选格子计算：

```text
总代价 f(n) = 已走代价 g(n) + 到终点的估计代价 h(n)
```

它优先探索“已经不太绕、并且看起来离终点更近”的格子，直到找到目标附近的格子，再反向还原为 `/plan`。因此它是**全局、离散栅格层面**的找路：负责绕墙、通过走廊、选大方向；它不直接决定速度，也不负责毫秒级躲开眼前突然出现的物体。

`tolerance: 0.25` 表示如果精确目标格不可达，规划器可接受距离目标 0.25 m 内的可达位置；`allow_unknown: true` 表示允许把未知区域作为可走候选。对于当前静态、已知地图，最常见的实际结果仍然是从当前位置到目标点的一条避开墙体的路径。

## 13. 项目中的 DWB 到底怎样选择速度？

当前实际配置的关键参数如下：

```yaml
max_vel_x: 0.35
max_vel_theta: 0.70
acc_lim_x: 1.0
acc_lim_theta: 1.8
vx_samples: 20
vtheta_samples: 24
sim_time: 1.3
linear_granularity: 0.05
angular_granularity: 0.025
critics: [RotateToGoal, Oscillation, BaseObstacle, GoalAlign,
          PathAlign, PathDist, GoalDist]
```

### 第一步：先确定这一帧“物理上可达到”的速度窗口

DWB 不会从 `0` 到最大速度任意跳选，而是看当前速度与加速度上限。控制频率是 20 Hz，即一帧约 0.05 秒；例如当前静止时，线加速度上限为 `1.0 m/s²`，本帧能增加的线速度最多约为：

```text
1.0 × 0.05 = 0.05 m/s
```

同理，角速度受 `acc_lim_theta: 1.8 rad/s²` 约束。这一限制形成 dynamic window（动态速度窗口）：速度变化不会突然从静止跳到 `0.35 m/s`，也不会瞬间猛打方向。

### 第二步：采样候选速度

在这个可达窗口和全局限制内，DWB 对线速度采样 20 个值、对角速度采样 24 个值，理论上最多形成：

```text
20 × 24 = 480 组 (vx, vtheta) 候选
```

差速机器人没有横移，因此 `vy = 0`。其中一组候选可能是：

```text
vx = 0.20 m/s,  vtheta = -0.15 rad/s
```

### 第三步：对每组候选作前向模拟

对每个 `(vx, vtheta)`，DWB 假设机器人在未来 `sim_time: 1.3 s` 内保持这组速度，按差速车运动模型向前积分，得到一条短预测轨迹。轨迹采样步长由 `linear_granularity: 0.05 m`、`angular_granularity: 0.025 rad` 控制。

直观例子：如果 `vx = 0.20 m/s`，不考虑转弯，1.3 秒大约前进 `0.26 m`。若同时有角速度，预测轨迹便是圆弧而不是直线。

### 第四步：先淘汰危险或不合法的轨迹

`BaseObstacle` critic 会把预测轨迹上的机器人 footprint 放进局部代价地图检查。该局部地图在 `odom` 坐标系下，以机器人为中心滚动，大小为 `6 m × 6 m`，分辨率 `0.05 m`；它使用 `/scan` 标记障碍、清除没有障碍的射线路径，并对障碍做 `0.42 m` 膨胀。

如果预测中小车 footprint 会进入致命障碍区域，这个候选会被判为不可用，不会因为其他分数很好而被选中。

### 第五步：对可行轨迹由多个 critic 打分

每个 critic 产生一个代价，再乘上 YAML 中对应的 `scale`，所有代价相加；总分较低的候选更好。

| Critic | 项目中的作用 | 配置权重 |
| --- | --- | --- |
| `BaseObstacle` | 远离局部代价地图中的障碍物 | 0.24 |
| `PathAlign` | 车头方向对齐全局路径前方 | 20.0 |
| `GoalAlign` | 车头方向对齐目标方向 | 14.0 |
| `PathDist` | 预测轨迹末端不要偏离全局路径 | 32.0 |
| `GoalDist` | 鼓励离最终目标更近 | 24.0 |
| `RotateToGoal` | 靠近目标后适当原地转向，满足最终朝向 | 24.0 |
| `Oscillation` | 避免左右反复切换、前后摆动 | 默认行为，无额外 scale |

这不是“某一个 critic 胜出”，而是综合取舍。例如离目标最近的候选若很贴近障碍物，会被淘汰或得高分；最安全的候选若完全偏离全局路径，也可能总分更差。

### 第六步：发布最低总代价的候选速度

DWB 选出总分最低的轨迹，将对应的 `(vx, vtheta)` 发布到 `/cmd_vel`。下一帧 0.05 秒后，它读取机器人真正移动后的新 TF、里程计和激光，重新做同一轮采样、预测与打分。

所以它不是先生成一条固定的局部轨迹并执行到底，而是持续“看一眼、预测、选一次、走一点、再看一眼”。这也是它能跟踪路径并应对小范围偏差的原因。

### 一个具体情境

全局路径要求先直行再左转，而激光发现右前方有墙：

1. 直行且右转的候选，可能碰撞或离障碍太近，被 `BaseObstacle` 淘汰/重罚。
2. 直行但轻微左转的候选，贴近路径且远离墙，`PathAlign`、`PathDist`、`BaseObstacle` 得分较好。
3. 原地左转的候选，很安全但暂时没有接近目标，`GoalDist` 得分不利。
4. DWB 综合分数后，多半选择“缓慢前进并轻微左转”。
5. 0.05 秒后又重新判断；接近拐角时，角速度会继续调整。

### 与项目后续工作的关系

当前 DWB 是静态地图导航基线：它使用当前时刻的局部障碍代价，主要处理眼前空间。PredictiveNav2 后续要做的是让预测模块告诉控制器“动态障碍物未来可能出现在哪里”，再把未来风险纳入控制器评价，而不是只看当前激光点。
