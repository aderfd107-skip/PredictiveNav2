# PredictiveNav2

PredictiveNav2 是一个基于 ROS 2 Jazzy / Nav2 的动态障碍物预测与风险感知导航系统。项目目标是将 2D LiDAR 动态目标检测、CV 卡尔曼跟踪、短期轨迹预测与 MPPI 动态风险评价集成到可复现实验中。

当前已完成单自车 Gazebo 仿真、圆形差速机器人 LiDAR/IMU/TF/odom 链路、SDF 派生真值地图、AMCL + DWB 静态导航基线、激光最终安全闸门、自动拐角预对齐/目标抢占，以及 LiDAR 聚类、CV 卡尔曼跟踪和带协方差的短期轨迹预测 baseline。原版 Nav2 MPPI、预测风险 MPPI Critic 和自动化 benchmark 尚未完成。完整设计见 [PROJECT_SPEC.md](PROJECT_SPEC.md)，日常完成进度见 [PROJECT_PROGRESS.md](PROJECT_PROGRESS.md)。

## 当前包

- `predictive_nav_description`：机器人 URDF/Xacro 与 RViz 显示。
- `predictive_nav_simulation`：Gazebo 动态导航实验场景、ROS-Gazebo bridge、地图制作和基础运行工具。
- `predictive_nav_bringup`：AMCL、Nav2 DWB 静态基线、拐角预对齐和稳定初始位姿确认。
- `predictive_nav_perception`：C++ LiDAR 聚类与动态障碍物候选观测。
- `predictive_nav_tracking`：CV 卡尔曼多目标跟踪与未来轨迹预测。

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

`predictive_dynamic_lab_nav.launch.py` 是当前静态安全基线入口。它默认启用 `scan_safety_guard.py`：近墙时阻止平移撞墙、但允许圆形底盘原地转向；它是最后防线，不是未来 MPPI 预测风险算法的替代品。静态阶段的验证结论与面试表达见 [docs/static_navigation_checkpoint.md](docs/static_navigation_checkpoint.md)。
