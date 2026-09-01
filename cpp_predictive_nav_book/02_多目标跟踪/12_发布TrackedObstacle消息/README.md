# 12：发布 TrackedObstacle 消息

## 先用一句人话说明

第 11 步的 `tracks_` 是 `tracking_node` 内存里的 C++ `std::vector<Track>`。其他节点看不到它：节点重启、换进程、将来新增 prediction 节点时，内部变量都不能直接共享。

所以这一步把“节点内部的一条 Track”翻译成所有 ROS 2 节点都认识的正式消息，并持续发布：

```text
/dynamic_obstacles/clusters
        ↓
tracking_node 内部 tracks_
        ↓
/dynamic_obstacles/tracks     ← 第 12 步新增
        ↓
第 03 模块的轨迹预测节点
```

这一步不改变 Track 算法，不重新做关联，不控制小车。它的职责只是把已经得到的状态用稳定、可复用的 ROS 接口交给下游。

## 为什么需要两条消息

新增文件在 `src/predictive_nav_msgs/msg/`：

```text
TrackedObstacle.msg       一条真实 Track 的状态
TrackedObstacleArray.msg  同一时刻的全部 Track
```

`TrackedObstacleArray` 有一份公共 header：

```text
header.stamp    这一批 Track 对应的 LiDAR 测量时间
header.frame_id 坐标系，当前必须是 odom
obstacles[]     当前存活的全部 Track
```

每条 `TrackedObstacle` 不重复放 header，因为数组中的每条 Track 都来自同一帧 cluster，时间和坐标系完全相同。这样消息更清楚，也不会产生重复数据。

## 每个字段是什么

```text
TrackedObstacle
├── track_id
├── pose
│   ├── pose.position.x / y
│   └── covariance[36]
├── twist
│   ├── twist.linear.x / y
│   └── covariance[36]
├── size.x / y
├── age
├── missed_frames
└── confidence
```

| 字段 | 来自内部 `Track` 的什么数据 | 含义 |
|---|---|---|
| `track_id` | `track_id` | 跨帧稳定 ID，不是本帧 cluster 下标。 |
| `pose.position.x/y` | `state[px/py]` | 当前滤波后的二维位置，单位米。 |
| `twist.linear.x/y` | `state[vx/vy]` | 当前滤波后的二维速度，单位 m/s。 |
| `pose.covariance` | `P` 的位置 2×2 部分 | 位置估计的不确定性。 |
| `twist.covariance` | `P` 的速度 2×2 部分 | 速度估计的不确定性。 |
| `size.x/y` | `size_x_m / size_y_m` | 本次匹配 cluster 的轴对齐尺寸。 |
| `age` | `age` | 该 Track 已经历的有效处理帧数。 |
| `missed_frames` | `missed_frames` | 已连续多少帧没有可信匹配。 |
| `confidence` | `1 / (1 + missed_frames)` | 仅表示观测新鲜程度的启发式值，不是经过统计标定的真实概率。 |

## 为什么协方差是 36 个数

`PoseWithCovariance` 和 `TwistWithCovariance` 是 ROS 常用的 3D 标准消息。它们的 covariance 都是 6×6 矩阵，因此有 `6 × 6 = 36` 个数。

但本项目目前只用 2D LiDAR 跟踪：

```text
可估计：x、y、vx、vy
不可估计：z、roll、pitch、yaw、vz、角速度
```

代码把内部 4×4 协方差中的位置部分放进 pose 的 x/y 位置，把速度部分放进 twist 的 vx/vy 位置。其余无法观测的维度不假装“误差为 0”，而是保留数值 0，同时设置很大的方差 `1e6`，表示“当前不知道它们”。

这样第 03 模块读取消息时，可以明确只使用 x/y 和 vx/vy，而不会误以为 tracker 已经知道障碍物的三维朝向。

## 发布规则

发布器使用：

```cpp
rclcpp::SensorDataQoS()
```

因为 Track 是高频、只关心最新状态的数据流。预测节点如果因为短暂负载错过旧 Track，通常应直接使用下一帧最新状态，而不是排队处理过期状态。

程序只在以下条件成立时发布：

```text
frame_id == odom
且 dt 是首帧，或有效 dt
```

原因是下游收到的 header 时间必须和 Track 状态一致。若这一帧 `dt` 倒退/过大，或 frame 不是 `odom`，程序不会把未正确推进的旧状态伪装成“当前时刻的状态”发出去。

## 本步代码做了什么

文件：[TrackedObstacle.msg](../../../src/predictive_nav_msgs/msg/TrackedObstacle.msg)、[TrackedObstacleArray.msg](../../../src/predictive_nav_msgs/msg/TrackedObstacleArray.msg)、[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)。

1. 在 `predictive_nav_msgs` 的 CMake 生成规则中加入两条消息；ROS 会生成 C++ 头文件。
2. `tracking_node` 创建 `/dynamic_obstacles/tracks` 发布器。
3. 每次真实 Track 生命周期完成后，调用 `publish_tracks()`。
4. 将内部状态 `[px, py, vx, vy]`、协方差、尺寸、年龄和 miss 信息复制到消息。
5. 每十帧输出一次 `published tracks` 日志，避免终端被高频消息刷满。

## 构建

这次同时改了消息包和 tracking 包，因此要构建它们的依赖链：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to predictive_nav_tracking
source install/setup.bash
```

我已在当前工作区执行过构建，`predictive_nav_msgs` 和 `predictive_nav_tracking` 都成功完成。

你也可以确认接口已经生成：

```bash
ros2 interface show predictive_nav_msgs/msg/TrackedObstacleArray
```

看到 `header` 和 `TrackedObstacle[] obstacles` 就正确。

## 如何运行和观察

开三个终端；每个终端都先执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

终端 1：

```bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

终端 2：

```bash
ros2 run predictive_nav_perception scan_info_node
```

终端 3：

```bash
ros2 run predictive_nav_tracking tracking_node
```

稳定后，终端 3 每十帧应包含：

```text
published tracks | topic=/dynamic_obstacles/tracks | frame=odom | ... | count=9
```

另开终端 4，观察一条完整消息：

```bash
ros2 topic echo --once /dynamic_obstacles/tracks
```

你会看到类似结构：

```text
header:
  frame_id: odom
obstacles:
- track_id: 1
  pose:
    pose:
      position:
        x: ...
        y: ...
  twist:
    twist:
      linear:
        x: ...
        y: ...
  age: ...
  missed_frames: 0
  confidence: 1.0
```

`covariance` 会很长，这是正常的 36 项数组；现在只需确认 x/y 与 vx/vy 对应字段存在，不需要手算全部数字。

再检查频率：

```bash
ros2 topic hz /dynamic_obstacles/tracks
```

它通常应接近 cluster 输入频率（约 10 Hz），但不能把具体数字当作硬性要求。

## 当前边界

- `confidence` 只是根据 `missed_frames` 得到的“观测新鲜程度”，不是“ID 一定正确”的概率；
- `size` 是当前 cluster 的轴对齐尺寸，不能当成真实障碍物精确长宽；
- 未匹配 Track 在过期前仍会发布预测状态，`missed_frames` 和较低 `confidence` 提醒下游它不是新观测；
- topic 已发布，但第 13 步才会在 RViz 画出 ID 和速度箭头；
- 此消息不会直接影响 `/cmd_vel` 或 Nav2。

## 本章必须懂的 ROS 2 / Nav2 知识

- C++ 内部变量不是 ROS 节点间接口；跨 package 通信要定义 `.msg` 并发布 topic。
- `header.stamp` 和 `header.frame_id` 是状态语义的一部分，不能只发 x/y 数字。
- publisher 和 subscriber 的 QoS 必须兼容；本项目 Track 流使用 `SensorDataQoS`。
- 第 03 模块只消费 `/dynamic_obstacles/tracks`，不应回头依赖 tracking 节点内部变量或 Gazebo 真值。

## 完成检查

- [ ] `colcon build --packages-up-to predictive_nav_tracking` 成功。
- [ ] `ros2 interface show predictive_nav_msgs/msg/TrackedObstacleArray` 显示 `header` 与 `obstacles`。
- [ ] tracking 节点出现 `published tracks` 日志。
- [ ] `ros2 topic echo --once /dynamic_obstacles/tracks` 能看到 `frame_id: odom`、`track_id`、位置、速度、年龄和 miss 信息。
- [ ] 理解 `confidence` 不是已经标定的概率，不能用于夸大算法可靠性。
