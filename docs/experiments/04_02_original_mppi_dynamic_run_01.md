# 04-02 原版 MPPI 动态方块基线：Run 01

## 目的

在尚未接入 `DynamicRiskCritic`、跟踪或预测输入时，记录 Nav2 原版 MPPI 对可见动态方块的实际反应。该记录是后续预测风险模块的对照组，不把一次运行的结果泛化为成功率结论。

## 固定条件

- ROS 2 Jazzy、Gazebo Harmonic、Nav2 原版 `nav2_mppi_controller::MPPIController`；
- 地图：`predictive_dynamic_lab_truth.yaml`；世界：`predictive_dynamic_lab.sdf`；
- 初始机器人位姿：`(-4.50, -6.80)` m；
- 动态 actor：`dynamic_obstacle_actor`，沿 `y=-1.00` m 在 `x=-6.80` 至 `-3.50` m 间往返，速度 `0.35 m/s`；
- 本次没有启用项目自定义 `DynamicRiskCritic`。

## 原始证据

原始 rosbag 保留在本机，**不提交到 Git**：

```text
artifacts/rosbags/04_02_original_mppi_dynamic_run_01
```

`ros2 bag info` 已确认：

- 录制时长：`188.186 s`；
- 总消息数：`12,518`；
- 已录制 `/scan`、`/odom`、`/cmd_vel`、`/plan`、`/local_costmap/costmap`、`/dynamic_obstacle/ground_truth_marker`；
- 机器人里程计起点：`(-4.50, -6.80)` m，录制结束位姿：约 `(-3.71, -0.98)` m；累计里程约 `6.834 m`；
- `/cmd_vel` 最大线速度约 `0.181 m/s`，有 `3,680` 条非零速度命令；
- actor marker 在录制区间内持续发布，证明动态方块控制器确实运行。

## 可得结论与边界

本次记录证明：原版 MPPI 在新的真值地图、移动方块和 LiDAR/local costmap 输入同时存在时，确实发布了连续控制命令并完成了一段实际移动。

本 rosbag **没有录到** `/navigate_to_pose/_action/status`，也未保存这次点击目标的坐标。因此不能仅凭 bag 断言最终 action 是 `SUCCEEDED`、`FAILED`，或发生了物理碰撞；这些应由下一次运行的 action 状态、目标坐标和 Gazebo 截图补充。不要把本次 run 写成“动态避障成功率”。

## 下次复现实验的补充项

运行前记录 RViz 目标坐标；运行后保存 action 的最终状态，并同时截取 Gazebo 与 local costmap。这样第 05 章才能计算到达率、最小间距和耗时。
