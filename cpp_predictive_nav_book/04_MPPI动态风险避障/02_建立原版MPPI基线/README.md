# 02：建立原版 MPPI 基线

> 本步目标：在当前地图和差速底盘上运行 Nav2 原版 MPPI；不加载 DynamicRiskCritic，也不使用 prediction 评分。

## 为什么必须先有 baseline

后续机器人减速、停住或绕开障碍，原因可能是 MPPI 原有静态避障，也可能是新的动态风险。
没有原版 MPPI 对照，就不能判断新 Critic 是否真的带来效果。

本项目保留独立文件：

```text
nav2_dwb.yaml             早期 DWB 配置，不覆盖
nav2_mppi_baseline.yaml   原版 MPPI 配置，不含 DynamicRiskCritic
```

## `FollowPath` 这段 YAML 在表达什么

当前 baseline 位于：

```text
src/predictive_nav_bringup/config/nav2_mppi_baseline.yaml
```

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      motion_model: "DiffDrive"
```

`FollowPath` 是 controller_server 中的实例名；`plugin` 是 MPPI C++ 实现；
`motion_model: DiffDrive` 表示候选轨迹符合差速底盘运动学，并不是重新改变底盘类型。

## 预测窗口与采样数量

当前配置包含：

```yaml
model_dt: 0.05
time_steps: 40
batch_size: 1000
```

所以 MPPI 每个控制周期向前模拟 `40 × 0.05 = 2.0 s`，与第三章默认 prediction horizon
便于比较。`batch_size: 1000` 表示一次比较约一千条候选轨迹，不是路径点数量。

## 如何运行

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup predictive_dynamic_lab_mppi.launch.py
```

在 RViz 选择 `2D Goal Pose` 发送短距离目标，并另开终端检查：

```bash
ros2 topic hz /cmd_vel
```

约 20 Hz 对应 `controller_frequency: 20.0`。频率存在只说明控制器在发布；是否能到达仍要看
action 结果、局部 costmap 与 Gazebo 画面。

## 常见问题

### 点击目标后没有速度

先确认 launch 使用的是 `nav2_mppi_baseline.yaml`，再看 controller_server 是否 active。
目标落在障碍物、AMCL/TF 异常或 action 已 aborted 都会造成没有有效速度；不要先改权重。

### 为什么 RViz 出现很多短线

这来自 `visualize: true` 和 `TrajectoryVisualizer`。它们是 MPPI 的机器人候选轨迹，不是
全局路径，也不是障碍物 prediction。

## 本步完成标准

- [ ] 新 YAML 被 launch 实际加载；
- [ ] DWB 配置未被覆盖；
- [ ] MPPI 能完成至少一个开阔区域目标；
- [ ] 已保存一次动态方块横穿时的原版行为、日志或 rosbag；
- [ ] YAML 中还没有 `DynamicRiskCritic`。

## 下一步

[03_观察MPPI候选轨迹和调试信息](../03_观察MPPI候选轨迹和调试信息/README.md)：把 RViz 中
的全局路径、局部候选轨迹和预测轨迹区分开。
