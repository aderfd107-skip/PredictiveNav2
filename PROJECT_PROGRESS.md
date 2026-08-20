# PredictiveNav2 项目进度

> **最后更新：2026-08-20**
>
> 本文件记录项目“实际已经完成并验证的内容”，不把设计目标当作既有功能。后续每次对项目做实质修改时，应同步更新“当前状态”和“变更记录”。完整技术设计见 [PROJECT_SPEC.md](PROJECT_SPEC.md)。

## 项目目标

PredictiveNav2 最终要实现：机器人用 2D LiDAR 发现并跟踪动态障碍物，预测其短期轨迹和不确定性，并把动态风险加入 Nav2 MPPI 的轨迹评分，使机器人能主动减速、等待或绕开可能发生冲突的动态目标。

最终链路：

```text
/scan → 障碍物检测/聚类 → 多目标跟踪（CV Kalman Filter）
      → 短期轨迹预测 → DynamicRiskCritic → Nav2 MPPI → /cmd_vel
```

## 当前状态

| 模块 | 状态 | 说明 |
|---|---|---|
| 机器人模型与传感器 | 已完成 | 差速底盘、2D LiDAR、IMU、TF、里程计。 |
| Gazebo 场景与静态地图 | 已完成 | 单机器人室内实验场景、地图和 ROS-Gazebo bridge。 |
| 基础激光安全过滤 | 已完成 | `scan_safety_guard.py` 对近距离障碍限速/停车；它不是预测风险算法。 |
| AMCL + Nav2 DWB 静态导航 | 已完成 | 已知地图定位、全局规划、DWB 局部控制、RViz 发导航目标。 |
| 动态障碍物 actor | 已完成 | 3 个可复位 Gazebo 方块，覆盖下方、中央、右上通道，可由 ROS 2 服务启停。 |
| LiDAR 聚类/动态目标检测 | 未开始 | 尚未从 `/scan` 输出障碍物观测。 |
| 多目标跟踪与 CV 卡尔曼滤波 | 未开始 | 尚无轨迹 ID、速度或协方差估计。 |
| 轨迹预测 | 未开始 | 尚无未来位置和不确定性预测消息。 |
| 原版 Nav2 MPPI baseline | 未开始 | 已安装相关依赖，但尚未配置和验证。 |
| DynamicRiskCritic | 未开始 | 尚未实现自定义 MPPI critic。 |
| Benchmark / 数据导出 | 未开始 | 尚无自动场景、指标统计或结果图表。 |

## 已验证的运行能力

运行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/PredictiveNav2/install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py
```

系统会启动 Gazebo、AMCL、Nav2 DWB 和 RViz。等待两套 lifecycle manager 输出 `Managed nodes are active` 后，在 RViz 中选择 **Nav2 Goal** 并在地图可通行区域点击目标点，机器人会规划路径并前往目标。

已完成一次端到端 smoke test：向 `/navigate_to_pose` 发送 `(5.8, -2.5)`，机器人从约 `(5.76, -3.80)` 行驶至约 `(5.81, -2.69)`，动作结果为 `SUCCEEDED`，恢复次数为 0。

## 变更记录

### 2026-08-19 — 静态导航基线完成最小验证

- 新建 `predictive_nav_bringup`，加入 AMCL + DWB、Nav2 lifecycle manager 和静态地图导航配置。
- 新增稳定初始位姿节点：等待 `/map` 与 `/scan`，并确认 AMCL 位姿对齐。
- RViz 改用 Nav2 专用 `Nav2 Goal`，而非只发布 `/goal_pose` 的默认工具。
- 加长 lifecycle manager 在 Gazebo 慢启动时的服务和 bond 等待时间。
- 修复辅助 Python 节点在 Ctrl+C 后重复 shutdown 的退出错误。
- 构建、参数加载和一次真实 `NavigateToPose` 仿真导航均通过。

### 2026-08-19 — 修复导航启动时 RViz 未弹出

- 原因：顶层 Nav2 launch 与被包含的 Gazebo launch 都使用 `use_rviz`；内部为了关闭其自身 RViz 设置的 `false` 泄漏到了顶层条件，导致 Nav2 RViz 没有被创建。
- 修复：将内部 Gazebo launch 放入 scoped launch group，使其参数不再覆盖顶层的 `use_rviz`。
- 影响：默认运行 `nav_baseline.launch.py` 会启动一个 Nav2 RViz；Gazebo 内部不重复启动 RViz。

### 2026-08-19 — 固化 RViz 导航操作入口

- 运行中验证：直接向 `/navigate_to_pose` 发送 `(5.8, -2.5)`，小车正常移动并返回 `SUCCEEDED`；AMCL、DWB、安全过滤与 Gazebo 执行链路均正常。
- 原因：用户界面中仍可选中 `Interact`，该工具的鼠标拖动只旋转/移动视角，不会发送导航目标；`2D Pose Estimate` 又会覆盖自动设置的 AMCL 初始位姿。
- 修复：将 `Nav2 Goal` 放在 RViz 工具栏第一位，使其默认选中；移除 `2D Pose Estimate`，防止破坏定位。

### 2026-08-19 — 修复 RViz Goal 未发送导航动作

- 根因：Jazzy 的 `nav2_rviz_plugins/GoalTool` 只发出内部 RViz 目标信号；`nav2_rviz_plugins/Navigation 2` 面板才会把该信号转为 `/navigate_to_pose` 动作。原配置遗漏了该面板，因此点击目标不会生成路径或速度命令。
- 修复：在默认 RViz 配置中加入 `Navigation 2` 面板。
- 验证依据：运行中直接向同一 `/navigate_to_pose` 动作服务器发送目标，机器人移动并返回 `SUCCEEDED`；修复后的 RViz 将使用同一动作接口。

### 2026-08-19 — 补充 ROS 2 导航学习笔记

- 新增 `docs/ros2_navigation_learning_notes.md`，记录本项目实际使用的 TF 链、LaserScan header、AMCL 初始位姿、Nav2 速度链路、常见排查方式与面试复习问题。
- 补充 Gazebo LiDAR、`ros_gz_bridge`、AMCL 粒子滤波和 `map → odom` 发布逻辑的一分钟面试示范回答。

### 2026-08-20 — 补充 Nav2 全链路与 DWB 学习笔记

- 在 `docs/ros2_navigation_learning_notes.md` 记录从 RViz 目标到 Gazebo 差速驱动的完整闭环，说明 `bt_navigator`、`planner_server`、DWB、`scan_safety_guard` 和反馈传感器的职责边界。
- 根据实际 `nav2_dwb.yaml` 记录 `NavfnPlanner` 的 `use_astar: true` 配置，以及 DWB 的速度窗口、20×24 候选采样、1.3 秒前向模拟、局部代价地图碰撞淘汰与 critic 综合评分过程。
- 本次为学习文档更新，未改变运行代码、参数或模块完成状态。

### 2026-08-20 — 修复 Nav2 基线启动中断

- 根因：`publish_initial_pose.py` 存在于安装目录，但源脚本缺少可执行权限；`--symlink-install` 下 ROS 2 因此无法将它作为节点启动，launch 在 Gazebo 启动后异常退出，RViz 与 Nav2 没有被创建。
- 修复：恢复脚本可执行权限并重新构建 `predictive_nav_bringup`。
- 验证：安装目录的可执行链接已通过 `test -x` 检查。

### 2026-08-20 — 修复导航时的 TF 父 frame 冲突

- 根因：`nav_baseline.launch.py` 启用 `world → odom` static anchor 的同时，AMCL 发布 `map → odom`；同一 `odom` 因而具有两个父 frame，导致地图定位、激光显示和控制轨迹错位。
- 修复：已知地图 AMCL 基线关闭 world odom anchor，仅保留 `map → odom → base_footprint`。
- 影响：必须重启 launch 后生效；独立 Gazebo 展示仍可使用 simulation launch 的默认 world anchor。

### 2026-08-20 — 补充动态预测前的坐标系学习笔记

- 在 `docs/ros2_navigation_learning_notes.md` 补充：为什么跨帧激光必须从 `lidar_link` 转到 `odom`、为什么首版短期 tracking 使用连续的 `odom` 而非直接在 `map` 中估速，以及 DWB 当前避障与未来轨迹预测的边界。
- 明确澄清：`map` 是固定全局 frame；AMCL 修正的是 `map → odom`，其不连续变化可能使转换后的全局目标坐标跳变，不能误当成障碍物自身运动。

### 2026-08-20 — 加入可控动态障碍物实验 actor

- 在 `dynamic_navigation_lab.sdf` 中加入橙色方块 `dynamic_obstacle_actor`，尺寸为 `0.45 × 0.45 × 0.95 m`。该模型不属于已保存的静态地图，因此不会破坏 AMCL 的地图匹配基准。
- 新增 `dynamic_obstacle_controller.py`：通过 `ros_gz_bridge` 的 `/world/dynamic_navigation_lab/set_pose` 服务，以 `0.35 m/s` 在 `(2.25, -3.70)` 与 `(2.25, -2.00)` 之间往返；提供 `/dynamic_obstacle_controller/start`、`/stop`、`/reset` 服务。
- 静态 baseline 与 mapping 默认关闭该 actor；动态试验通过 `enable_dynamic_obstacle:=true` 显式启用，避免污染静态对照组。
- 验证：`predictive_nav_simulation` 构建通过；运行中 Gazebo set_pose bridge 成功连接，模型 3 秒内从 `y=-2.4025` 移至 `y=-3.6300`，控制服务与 `/scan` 均已出现。

### 2026-08-20 — 加入动态 actor 的 RViz 真值调试覆盖层

- `dynamic_obstacle_controller.py` 在 `/dynamic_obstacle/ground_truth_marker` 发布半透明橙色框，坐标在 `odom`，仅用于核对 Gazebo actor 与激光观测是否一致。
- 默认 RViz 加入该 Marker 显示；红色 `/scan` 点仍是后续算法唯一允许使用的传感器观测，真值话题明确禁止接入检测、跟踪和预测逻辑。

### 2026-08-20 — 扩展为三条动态障碍物路线

- 新增 `dynamic_obstacle_center_actor` 与 `dynamic_obstacle_upper_actor`，与原下方通道 actor 分别覆盖中央主通道、右上房间和下方通道，避免机器人选择不同目标时没有动态场景可测。
- 控制节点参数化 actor 名称；三个 actor 各自拥有独立的 start/stop/reset 服务，并在同一个 RViz 真值 Marker 话题中以独立 namespace 发布方框。
- 静态 baseline 默认仍关闭全部 actor；动态试验一次启用三者。

### 2026-08-20 — 同步项目总体规格

- 更新 `PROJECT_SPEC.md` 的代码库审查、动态场景、ROS 接口、TF 风险和实施状态，使其与当前单机器人、3 个动态 actor、AMCL + DWB baseline 的实际状态一致。
- 明确仍未实现的边界：C++ LiDAR 聚类、跟踪、预测、原版 MPPI、DynamicRiskCritic 和自动化 benchmark 均保持 `Not Started`，没有因场景完成而被提前标记。

### 后续记录格式

每次实质改动追加一项，至少包含：日期、改动内容、影响模块、验证结果，以及是否改变了本文件的模块状态。
