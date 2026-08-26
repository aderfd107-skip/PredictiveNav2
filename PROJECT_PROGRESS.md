# PredictiveNav2 项目进度

> **最后更新：2026-08-26**
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
| LiDAR 聚类/动态目标检测 | 已开始（已发布且可视化） | `predictive_nav_perception/scan_info_node` 已用 C++ 订阅 `/scan`、筛除无效量程、生成 `odom` 点并做顺序欧氏聚类，发布 `/dynamic_obstacles/clusters` 和仅调试用的 RViz MarkerArray；它仍只是几何候选观测，尚未判断动态性。 |
| 多目标跟踪与 CV 卡尔曼滤波 | 已开始（单目标速度实验已实现） | `predictive_nav_tracking/tracking_node` 已订阅 cluster、验证 `dt`，并可在受控参考区域从真实 cluster 观测中连续选定一个目标，用两帧差分打印朴素速度；尚未分配稳定 ID、创建实际 Track、做多目标关联、KF 估计或发布 tracks。 |
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

### 2026-08-26 — 单目标连续观测与朴素速度实验

- `tracking_node` 新增一个仅用于教学/验证的单目标实验：在参数化 `odom` 参考区域中，从 `/dynamic_obstacles/clusters` 的真实 LiDAR 聚类结果选出最近观测，并根据连续有效帧的 `(centroid_now - centroid_previous) / dt` 打印 `vx`、`vy` 与速度大小。
- 它不订阅 Gazebo 真值 topic；参考区域只用于固定本次实验的选择范围。目标丢失或 `dt` 无效时会重置上一帧记录，避免跨异常时间计算速度。
- 此功能不是 Track 或多目标数据关联实现：不创建 ID、不维护真正轨迹、不估计协方差、不发布 `/dynamic_obstacles/tracks`。场景运行验证待用户按第 06 步执行确认。

### 2026-08-26 — 创建多目标跟踪 C++ 包骨架

- 新建 `predictive_nav_tracking`，包含 `package.xml`、`CMakeLists.txt` 与最小 `tracking_node`；声明 `rclcpp`、`predictive_nav_msgs` 和 Eigen 构建依赖，为后续 cluster 订阅与 CV 卡尔曼滤波预留接口。
- 当前节点只输出启动提示，不读取 `/dynamic_obstacles/clusters`，不创建轨迹 ID，也不产生任何跟踪结论。
- 验证：`colcon build --packages-select predictive_nav_tracking` 成功；`ros2 run predictive_nav_tracking tracking_node` 已输出预期启动日志。

### 2026-08-26 — 接入 cluster 消息到跟踪节点

- `tracking_node` 以 `SensorDataQoS` 订阅 `/dynamic_obstacles/clusters`，每十帧打印输入 header、cluster 数量和第一个 cluster 的中心、尺寸、点数；若 frame 不是 `odom` 会明确告警。
- 此改动只验证模块间消息接口，不创建 track、不关联观测、不估计速度，也不发布 `/dynamic_obstacles/tracks`。
- 验证：`colcon build --packages-select predictive_nav_tracking` 成功；完整 Gazebo 场景下的持续接收日志待用户按第 03 步运行确认。

### 2026-08-26 — 定义跨帧 Track 状态结构

- `tracking_node` 新增 `Track`：预留稳定 ID、`[px, py, vx, vy]` 状态、4×4 协方差、尺寸、首次/最近观测时间、`age` 和 `missed_frames`；节点成员 `tracks_` 用于后续跨 callback 持久保存轨迹。
- 本步不从 cluster 创建 Track，`tracks_` 预期为空，日志显示 `active_tracks=0`；状态和协方差当前仅为可构建的占位初值，不代表真实估计。
- 验证：`colcon build --packages-select predictive_nav_tracking` 成功。

### 2026-08-26 — 按消息时间戳验证 tracking dt

- `tracking_node` 新增基于连续 `ObstacleClusterArray.header.stamp` 的真实 `dt` 计算；首帧、非正/倒退时间和超过 `max_dt_s`（默认 0.50 s）的异常大间隔分别记录状态和诊断计数。
- 当前仅验证时间输入；异常 dt 会在未来跟踪循环中被拒绝，尚未用于速度、卡尔曼预测或 track 更新。
- 验证：`colcon build --packages-select predictive_nav_tracking` 成功；稳定 Gazebo 场景下 dt 约 0.1 s 的运行验证待用户完成。

### 2026-08-25 — 加入 cluster 的 RViz 验证覆盖层

- 感知节点根据自身 `/scan` 聚类结果，在 `/dynamic_obstacles/cluster_markers` 发布 MarkerArray：青色轴对齐方框表示尺寸，红色小球表示中心；每帧清理旧 Marker，避免 cluster 数量减少后出现幽灵障碍物。
- Nav2 默认 RViz 配置新增 `Scan-Derived Clusters (Debug)` MarkerArray display；它与现有橙色 Gazebo 真值调试 Marker 分离，真值仍不参与算法输入。
- 验证：`predictive_nav_perception` 与 `predictive_nav_bringup` 构建成功。运行时 RViz 对照待用户按第 09 步启动动态场景后完成。

### 2026-08-25 — 发布 LiDAR cluster 观测接口

- 新建 `predictive_nav_msgs` 接口包，定义 `ObstacleCluster`（中心、二维尺寸、点数）和带公共时间戳/坐标系的 `ObstacleClusterArray`。
- `scan_info_node` 以 `SensorDataQoS` 向 `/dynamic_obstacles/clusters` 发布每帧聚类结果；输出 header 严格使用原始 scan 时间戳和 `odom` frame，不读取 Gazebo 真值。
- 验证：`colcon build --packages-up-to predictive_nav_perception` 成功，新增消息包与感知包均完成；`ros2 interface show predictive_nav_msgs/msg/ObstacleClusterArray` 显示预期字段。Gazebo 运行时 topic 回显待用户按第 08 步完成。

### 2026-08-25 — 实现顺序欧氏 LiDAR 聚类

- `scan_info_node` 按相邻原始 beam index 和二维欧氏距离将 `odom` 跟踪点分为候选 cluster；对每组计算点数、质心和轴对齐尺寸。
- 新增距离阈值、最小/最大 cluster 点数参数，并分别统计过小噪声组与过大结构组；当前 cluster 仅内部生成并节流打印，尚未发布 ROS 消息。
- 验证：C++ 节点重新编译、链接和安装成功。运行时场景在验证时已停止、`/scan` 不存在，因此 cluster 输出待下次启动 Gazebo 后按第 07 步执行验证。

### 2026-08-25 — 将有效 LiDAR 点转换到 odom

- `scan_info_node` 将每束有效 LaserScan 依据原始 beam index、`angle_min` 和 `angle_increment` 转为 `lidar_link` 二维点，再通过 `odom ← lidar_link` TF 转为跟踪点。
- 新增 `tf2`、`tf2_ros`、`tf2_geometry_msgs` 与 `geometry_msgs` 依赖；TF 查询严格使用 scan 的原始时间戳。查不到该时刻变换时丢弃当前帧并计数，绝不复用旧变换。
- 修复运行验证中发现的 TF2 等待警告：感知回调不再阻塞等待 TF，而是立即查询并安全处理暂时不可用的帧。
- 验证：在 Gazebo 运行场景中，单帧 `ranges=720`、`kept=719`、`lidar_points=719`、`tracking_points=719`、`tf_failures=0`。

### 2026-08-24 — 实现 LaserScan 有效距离筛选

- `scan_info_node` 对每帧 `/scan` 的所有 `ranges` 执行有限值与有效量程检查；保留原光束下标和距离，为后续极坐标转二维点准备。
- 新增 `min_detection_range`、`max_detection_range` 参数；运行时取它们与传感器声明量程的交集，分别统计非有限和范围外的丢弃原因。
- 验证：节点重新编译、链接和安装成功；尚未实现二维坐标、TF 或聚类。

### 2026-08-24 — 加入最小 C++ LaserScan 订阅节点

- `predictive_nav_perception` 新增 `scan_info_node`，使用 `SensorDataQoS` 订阅 `/scan`；每收到 10 条 LaserScan 消息打印 frame、时间戳、距离数量、角度与量程摘要。
- 更新 CMake 与 package 依赖，加入 `rclcpp`、`sensor_msgs`，并安装可执行节点。
- 节点仅用于验证真实 LiDAR 输入；它不读取 Gazebo 真值、不发布控制命令、不做点过滤或聚类。
- 验证：`colcon build --packages-select predictive_nav_perception` 成功，1 个包完成。

### 2026-08-24 — 创建 C++ 感知包骨架

- 新建 `predictive_nav_perception`，包含 `package.xml`、`CMakeLists.txt`、`include/` 与 `src/` 目录，作为后续 LiDAR 感知 C++ 实现的唯一源码位置。
- 当前包故意未加入节点、消息、算法或 ROS 运行依赖；下一步将从最小 `/scan` 订阅节点开始逐项加入。
- 验证：`colcon build --packages-select predictive_nav_perception` 成功，1 个包完成。

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
