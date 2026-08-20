# Nav2 静态 DWB Baseline

该入口是 PredictiveNav2 的非预测对照组：在保存的 `dynamic_navigation_lab` 地图中使用 AMCL 定位，并以 DWB 执行 Nav2 导航。它不订阅或发布任何动态障碍物预测；后续原版 MPPI 与 `DynamicRiskCritic` 实验必须沿用相同地图、起点、目标和速度上限。

## 启动

先安装 Jazzy 的 Navigation2（至少包含 AMCL、DWB、planner、behavior、BT navigator 和 lifecycle manager），然后构建工作区：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_description predictive_nav_simulation predictive_nav_bringup
source install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py
```

默认在 `(5.8, -3.85, 1.5708)` 生成机器人，使用 `dynamic_navigation_lab.yaml`。初始位姿节点会等待 `/map` 与 `/scan`，然后发布 `/initialpose`，并在 `/amcl_pose` 连续满足位置与朝向阈值后停止。

## 验收

1. RViz 的 fixed frame 为 `map`，地图、机器人模型、TF 和 `/scan` 可见。
2. `/amcl_pose` 与 Gazebo 中机器人位置对齐，且 `map → odom → base_footprint` 连通。
3. 用 RViz 的 `Nav2 Goal` 连续发送至少三个静态可达目标；机器人能到达，路径不穿墙、不持续振荡。
4. 保持 `enable_scan_safety_guard` 的默认启用状态，以维持 `/cmd_vel → /cmd_vel_safe` 的仿真执行链路。

`Nav2 Goal` 会调用 Nav2 的导航动作；它不同于 RViz 默认的 `2D Goal Pose`，后者只发布普通的 `/goal_pose` 话题，不能保证触发导航。Jazzy 中 GoalTool 需配合 `Navigation 2` 面板才能把目标发送给 `NavigateToPose`，该面板已包含在配置中。该 RViz 配置会默认选择 `Nav2 Goal`，并特意不提供 `2D Pose Estimate`：仿真出生位姿由 bringup 自动设置，手动重设会使 AMCL 的 `map → odom` 对齐错误。Gazebo 启动时资源加载可能较慢，bringup 已为生命周期管理器配置较长的服务与 bond 等待时间；应等待其报告受管节点已激活，再发送目标。

该 baseline 只用于静态回归和后续对照；不得将临时 `scan_safety_guard` 的行为计入 DynamicRiskCritic 效果。

## 最小验证记录

2026-08-19：三包构建和启动参数加载通过；定位与导航 lifecycle manager 均进入 Active。对 `(5.8, -2.5)` 的 `NavigateToPose` 请求返回 `SUCCEEDED`，零 recovery。这是单目标 smoke test，不替代本页要求的三目标静态回归。
