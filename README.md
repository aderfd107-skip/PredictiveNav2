# PredictiveNav2

PredictiveNav2 是一个基于 ROS 2 Jazzy / Nav2 的动态障碍物预测与风险感知导航系统。项目目标是将 2D LiDAR 动态目标检测、CV 卡尔曼跟踪、短期轨迹预测与 MPPI 动态风险评价集成到可复现实验中。

当前已完成的是单自车 Gazebo 仿真、差速机器人 LiDAR/IMU/TF/odom 链路、地图制作工具和临时的扫描急停保护，以及已经端到端验证的 AMCL + DWB 已知地图 Nav2 baseline。动态检测、跟踪、预测、原版 MPPI、MPPI Critic 和 benchmark 尚未实现。完整设计见 [PROJECT_SPEC.md](PROJECT_SPEC.md)，日常完成进度见 [PROJECT_PROGRESS.md](PROJECT_PROGRESS.md)。

## 当前包

- `predictive_nav_description`：机器人 URDF/Xacro 与 RViz 显示。
- `predictive_nav_simulation`：Gazebo 动态导航实验场景、ROS-Gazebo bridge、地图制作和基础运行工具。
- `predictive_nav_bringup`：AMCL、Nav2 DWB 静态基线与稳定初始位姿确认。

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_description predictive_nav_simulation
source install/setup.bash
ros2 launch predictive_nav_simulation dynamic_navigation_lab.launch.py
```

当前标准接口为 `/scan`、`/odom`、`/imu`、`/tf`、`/cmd_vel` 和 `/cmd_vel_safe`。详细检查和 SLAM 建图步骤见 [docs/run_checklist.md](docs/run_checklist.md)。

安装 Nav2 后，可启动静态 DWB 对照组：

```bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py
```

`scan_safety_guard.py` 仅用于手动仿真的基础限速。它可通过 `enable_scan_safety_guard:=false` 关闭，且不会作为未来 MPPI 风险算法或 benchmark 的一部分。
