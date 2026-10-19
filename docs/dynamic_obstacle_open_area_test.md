# 新地图动态障碍物：第一轮测试

本测试使用新的 `predictive_dynamic_lab.sdf` 与配套真值地图。动态 actor 的写法参考旧
`dynamic_navigation_lab.sdf`：模型定义在 SDF 内，再由 `set_pose` 控制器做确定性往返。
静态基线中，三个 actor 位于地图外；本测试的专用 launch 才会将它们移入场景。
它验证“动态 actor 真实出现、被 LiDAR 观察、进入 cluster/track/prediction 链路”；它**不**测试预测风险避障。

## 场景固定条件

新地图包含三个 actor；本轮优先观察中央 actor `dynamic_obstacle_center_actor`：

```text
(-1.60, 0.00)  <---->  (1.20, 0.00),  0.30 m/s
```

三条路线为：

| actor | 路线 | 速度 |
|---|---|---:|
| `dynamic_obstacle_actor` | `(-6.80, -1.00) ↔ (-3.50, -1.00)` | `0.35 m/s` |
| `dynamic_obstacle_center_actor` | `(-1.60, 0.00) ↔ (1.20, 0.00)` | `0.30 m/s` |
| `dynamic_obstacle_upper_actor` | `(2.20, 2.20) ↔ (5.40, 2.20)` | `0.28 m/s` |

它们由 Gazebo `set_pose` 服务移动；真值地图生成器只读取静态墙体模型，因此不会把 actor 写入地图。橙色半透明方块仅是 RViz 真值调试覆盖层。算法输入仍必须是 `/scan`，不能订阅真值 Marker。

## 终端 1：启动 Nav2、Gazebo 和中央 actor

先停止旧的静态 launch，再执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch predictive_nav_bringup predictive_dynamic_lab_dynamic.launch.py
```

等待 RViz 的 `Localization` 与 `Navigation` 均显示 `active`。启动终端还应出现三个类似：

```text
Dynamic actor 'dynamic_obstacle_center_actor' is spawned and moving.
```

在 Gazebo 中应看到三个移动方块；本轮导航与观察先聚焦中央开阔区的那个。若出现
`Gazebo rejected set_pose ...`，先确认启动的是本页的
`predictive_dynamic_lab_dynamic.launch.py`，不要继续排查 cluster 或 prediction。

## 终端 2：LiDAR 聚类

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run predictive_nav_perception scan_info_node \
  --ros-args -p use_sim_time:=true
```

## 终端 3：多目标跟踪

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run predictive_nav_tracking tracking_node \
  --ros-args -p use_sim_time:=true
```

## 终端 4：轨迹预测

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run predictive_nav_prediction prediction_node \
  --ros-args -p use_sim_time:=true
```

## 终端 5：只读检查

先检查三段 ROS 链是否全部存在：

```bash
ros2 topic info /dynamic_obstacles/clusters -v
ros2 topic info /dynamic_obstacles/tracks -v
ros2 topic info /dynamic_obstacles/predictions -v
```

预期发布者依次为 `scan_info_node`、`tracking_node`、`prediction_node`。

再观察一次正式预测消息：

```bash
ros2 topic echo --once /dynamic_obstacles/predictions
```

正常时可看到轨迹点的时间偏移为 `0.2, 0.4, ..., 2.0 s`，每个未来位置带二维协方差。

## 通过标准与边界

本轮通过需同时满足：

- Gazebo 的 central actor 往返运动；
- `/scan` 可观察到它；
- `/dynamic_obstacles/clusters → tracks → predictions` 三段均持续有数据；
- RViz 中的 cluster、track 和预测轨迹随 actor 运动而更新；
- actor 真值 Marker 只用于目视对照，不进入任何算法订阅。

即使此时机器人给 actor 让路，也不能宣称“预测避障已完成”：当前 DWB 仍只使用自身 costmap，尚未消费 `/dynamic_obstacles/predictions`。下一阶段才会把预测风险接入 MPPI / 自定义 critic。
