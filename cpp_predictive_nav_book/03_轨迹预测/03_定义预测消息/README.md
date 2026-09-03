# 03：定义预测消息

> 本步目标：定义预测模块的输出数据格式，让后续代码能明确表达：**哪个目标、从什么时候开始、未来哪一刻、可能在哪里、这个判断有多不确定**。

本步只新增 ROS 2 消息接口，不订阅 `/dynamic_obstacles/tracks`，也不计算 CV 预测。先把模块之间传递数据的“合同”写清楚，再写算法。

## 1. 为什么不能只发布一个未来坐标？

假设当前时刻是 `10.0 s`，一个目标位于 `(2.0, 1.0)`，速度为 `(-0.5, 0.0) m/s`。对避障来说，只知道“它将来在 `(1.5, 1.0)`”没有意义：这是 `1 s` 后，还是 `10 s` 后？

风险评估需要将“小车在某个时刻的候选位置”和“障碍物在同一时刻的预测位置”对齐比较。因此一条预测不是一个点，而是一串按时间排列的未来点：

```text
预测基准时刻：10.0 s（header.stamp）

0.2 s 后  →  10.2 s  →  未来位置 1
0.4 s 后  →  10.4 s  →  未来位置 2
0.6 s 后  →  10.6 s  →  未来位置 3
```

## 2. 新增的两种消息

```text
TrackedObstacleArray                 PredictedTrajectoryArray
（第二模块的当前状态）                 （第三模块的未来预测）

header: 10.0 s, odom        →       header: 10.0 s, odom
  ├─ track id=7                         ├─ trajectory id=7
  │    position, velocity               │    0.2 s → future pose
  └─ track id=12                        │    0.4 s → future pose
                                        │    0.6 s → future pose
                                        └─ trajectory id=12 ...
```

### `PredictedTrajectory.msg`：一个目标的一串未来位置

文件位置：[PredictedTrajectory.msg](../../../src/predictive_nav_msgs/msg/PredictedTrajectory.msg)。

```text
uint32 track_id
builtin_interfaces/Duration[] time_offsets
geometry_msgs/PoseWithCovariance[] poses
geometry_msgs/Vector3 size
float32 confidence
```

字段含义：

| 字段 | 含义 | 单位 / 注意点 |
| --- | --- | --- |
| `track_id` | 从第 02 模块继承的稳定目标编号 | 例如 `7`；不是数组下标 |
| `time_offsets` | 每个未来点距“现在”的时间 | `Duration`，例如 `0.2 s` |
| `poses` | 同一未来时刻的位置、姿态和协方差 | 位置单位 m；协方差单位对应 m² |
| `size` | 障碍物尺寸 | 从 Track 复制，单位 m |
| `confidence` | Track 的新鲜度启发式值 | 不是“预测 90% 正确”的概率 |

最重要的对应关系是：

```text
time_offsets[i]  <──必须对应──>  poses[i]
```

也就是说两个数组长度必须相同。若 `time_offsets[1] = 0.4 s`，那么 `poses[1]` 必须恰好是 0.4 秒后的预测结果，而不能是别的时刻。

### 为什么用 `Duration`，而不是直接存 `float32` 秒？

ROS 2 的 `builtin_interfaces/Duration` 表示一个时间长度，由秒和纳秒组成。它避免了大家猜测“这个浮点数究竟是秒、毫秒，还是相对于哪个时刻”的问题，也能减少浮点累计误差造成的语义混乱。

例如概念上 `0.2 s` 是：

```text
sec: 0
nanosec: 200000000
```

你现在不需要手写它；第 05 步会用 C++ 从预测时间轴自动生成这些 offset。

### `PredictedTrajectoryArray.msg`：同一帧所有目标的预测

文件位置：[PredictedTrajectoryArray.msg](../../../src/predictive_nav_msgs/msg/PredictedTrajectoryArray.msg)。

```text
std_msgs/Header header
PredictedTrajectory[] trajectories
```

`header` 放在外层数组，和第二模块的 `TrackedObstacleArray` 一样：同一帧所有预测使用同一个基准时刻和坐标系，无须在每个目标里重复保存。

| 字段 | 本项目约定 |
| --- | --- |
| `header.stamp` | 输入 `/dynamic_obstacles/tracks` 的测量时间；也就是 offset 的 `0 s` |
| `header.frame_id` | `odom`，所有预测 `pose` 都在此坐标系表示 |
| `trajectories` | 这一帧每个有效 Track 对应的一条未来轨迹 |

例如 `header.stamp = 10.0 s` 且 `time_offsets[0] = 0.2 s`，则第一预测点代表 **10.2 s**，不是“发布消息的电脑当前时钟时间 + 0.2 s”。这一区分在 rosbag 回放和真实机器人延迟场景中非常重要。

## 3. 为什么 `poses` 是 `PoseWithCovariance`？

如果只用 `geometry_msgs/Pose`，下游只能知道“预测位置是 `(x, y)`”，却不知道该信多少。现实中预测越远，误差通常越大：

```text
现在的估计：较确定
0.2 s 后：略不确定
1.0 s 后：更不确定
```

`PoseWithCovariance` 同时保存姿态和一个 6×6 协方差矩阵。第 07 步会计算其中 x/y 位置相关的数值；目前你只要建立这个概念：**预测位置不是一个绝对事实，而是带误差范围的估计。**

和第 02 模块一样：

- x/y 的方差在协方差数组的对角线位置；
- x/y 的相关性可在非对角位置表示；
- 未观测的 z、roll、pitch 等维度将按约定给合理的大对角方差，而不是把所有非对角元素随意填大。

## 4. 为什么不在每条轨迹里再放一个 `Header`？

你可能会想到每个 `PredictedTrajectory` 都写一个 `Header`。但当前设计中，一帧所有 trajectory 都来自同一个 `/tracks` 消息，因此它们天然共享时间和坐标系。把 header 放在外层有三个好处：

- 不会让某一条轨迹悄悄处于不同坐标系或时间基准；
- 消息更小、语义和现有 `TrackedObstacleArray` 一致；
- 第 04 模块可以一次检查整个 prediction frame 的时间有效性。

如果以后出现不同来源、不同时间的异步预测，才可能需要每条轨迹单独 header；当前项目不需要提前复杂化。

## 5. 实际执行了哪些改动？

新增：

```text
src/predictive_nav_msgs/msg/PredictedTrajectory.msg
src/predictive_nav_msgs/msg/PredictedTrajectoryArray.msg
```

并更新了 `predictive_nav_msgs` 的：

- `CMakeLists.txt`：把两份消息交给 `rosidl_generate_interfaces()` 生成 C++ 接口；
- `package.xml`：声明 `builtin_interfaces` 依赖；
- `PROJECT_SPEC.md`：同步外层共同 `Header` 的实际接口约定。

## 6. 如何验证消息已生成？

在终端执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_msgs
source install/setup.bash

ros2 interface show predictive_nav_msgs/msg/PredictedTrajectory
ros2 interface show predictive_nav_msgs/msg/PredictedTrajectoryArray
```

第一条命令应能看到 `track_id`、`time_offsets`、`poses`、`size`、`confidence`；第二条应能看到 `header` 和 `trajectories`。

注意：此时执行下面命令会看不到话题，这是**正常的**，因为第 09 步才创建发布者：

```bash
ros2 topic echo /dynamic_obstacles/predictions
```

## 7. 本步完成标准

- [ ] `colcon build --packages-select predictive_nav_msgs` 成功。
- [ ] 两个 `ros2 interface show` 命令能显示接口。
- [ ] 能说清 `time_offsets[i]` 与 `poses[i]` 是一一对应的。
- [ ] 能说清 offset 是相对于外层 `header.stamp`，不是相对于电脑当前时间。
- [ ] 知道 prediction 仍然没有控制小车，也还没有开始做 CV 计算。

## 本章必须懂的 ROS 2 / C++ 知识

- `.msg` 文件是跨语言消息合同；`rosidl_generate_interfaces()` 会据此生成 C++、Python 等接口代码。
- 数组字段通过相同下标表达一一对应关系时，必须在代码中检查长度一致性。
- `Header` 同时给出时间戳和坐标系；坐标和时间语义比“字段能传过去”更重要。
- `PoseWithCovariance` 让下游知道预测的不确定性，第 07 步才会真正传播它。

## 下一步

[04_订阅track并打印](../04_订阅track并打印/README.md)：先让 `prediction_node` 订阅第二模块的输出并只打印字段，确认真实输入连通后，再写预测时间轴和 CV 公式。
