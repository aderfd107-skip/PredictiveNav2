# 09：发布预测结果

## 本步完成后，数据流终于连起来了

第 06、07 步已经能在函数内部计算某一个 Track 的未来位置和位置不确定性；第 08 步又筛掉了过期、异常或不稳定的 Track。本步把这些内部 C++ 数据真正变成 ROS 2 消息，发布给下游模块：

```text
/dynamic_obstacles/tracks
        ↓
质量闸门 + CV 传播 + 协方差传播
        ↓
/dynamic_obstacles/predictions
        ↓
第 10 步 RViz 显示
        ↓
第 04 模块 MPPI 动态风险评估
```

发布话题是：

```text
/dynamic_obstacles/predictions
类型：predictive_nav_msgs/msg/PredictedTrajectoryArray
```

本步仍然**不会**发布 `/cmd_vel`，也不会修改 Nav2 的代价地图、控制器或 MPPI 参数。它只是把“动态障碍物未来可能在哪里”交给下一层。

## 为什么要发布“数组”而不是一条轨迹

一个激光帧里可能同时看到多个障碍物，因此一帧输出是：

```text
PredictedTrajectoryArray
├── header                 # 这批预测共同的起始时刻和坐标系
└── trajectories[]         # 每个合格 Track 对应一条未来轨迹
    ├── track_id
    ├── time_offsets[]
    ├── poses[]
    ├── size
    └── confidence
```

每条 `PredictedTrajectory` 里的 `time_offsets[i]` 与 `poses[i]` 是一对：

```text
header.stamp = 100.0 s

time_offsets[0] = 0.2 s  ↔ poses[0]：目标在 100.2 s 时的预测位置
time_offsets[1] = 0.4 s  ↔ poses[1]：目标在 100.4 s 时的预测位置
...
time_offsets[9] = 2.0 s  ↔ poses[9]：目标在 102.0 s 时的预测位置
```

这里的 `header.stamp` **不是节点计算完消息的电脑时间**，而是输入 Track 对应的观测时间。这样下游才知道预测轨迹真正对应现实中的哪个时刻。

## 每个字段从哪里来

| 输出字段 | 来源 | 含义 |
| --- | --- | --- |
| `header` | 输入 `TrackedObstacleArray.header` | 保持相同的测量时刻和 `odom` 坐标系。 |
| `track_id` | `TrackedObstacle.track_id` | 保持同一目标的身份编号。 |
| `time_offsets` | 第 05 步时间轴 | 例如 0.2 s 到 2.0 s。 |
| `poses[i].position` | 第 06 步 CV 位置传播 | `x + vx × t`、`y + vy × t`。 |
| `poses[i].covariance` | 第 07 步协方差传播 | 每个未来时刻自己的位置不确定性。 |
| `size` | 输入 Track 的 cluster 尺寸 | 后续风险评估不能只把障碍物当作一个点。 |
| `confidence` | 输入 Track | 当前只是新鲜度启发式，不是严格概率。 |

## 2×2 协方差如何放入 ROS 的 6×6 Pose 协方差

本项目只预测平面位置 `(x, y)`，所以内部得到的是：

```text
[ Pxx  Pxy ]
[ Pyx  Pyy ]
```

但 ROS 标准 `PoseWithCovariance` 使用 6×6 协方差，变量顺序是：

```text
[x, y, z, roll, pitch, yaw]
```

因此代码把二维位置块放到 6×6 的左上角：

```text
covariance[0] = Pxx    covariance[1] = Pxy
covariance[6] = Pyx    covariance[7] = Pyy
```

`z`、roll、pitch、yaw 在 2D 激光跟踪中不可观测，所以它们的**对角线方差**设为 `1e6`；其余交叉项保持 0。这里沿用了第 02 章的原则：不知道某个变量，不等于它和 x/y 存在巨大相关性。

预测 Pose 的朝向暂时设为单位四元数：

```text
orientation.w = 1.0
```

这不代表障碍物真的朝向 x 轴，只表示当前项目的动态障碍物模型只关心平面位置和尺寸，不把目标朝向作为预测状态。

## “空预测帧”是什么意思

这是本步很重要的工程约定。

若收到的 Track 帧本身新鲜、`frame=odom`，但其中没有任何 Track 通过质量闸门，例如都刚出生或速度异常，节点仍会发布：

```text
PredictedTrajectoryArray
├── header：有效
└── trajectories：[]
```

它的含义是：**这一时刻确实处理过输入，但没有可信轨迹可供避障使用。**

这与“不发布任何消息”不同：不发布可能表示 prediction 节点死了、网络断了，或上游完全停止。第 08 步的 watchdog 会对后者报警；未来风险模块还应根据 `header.stamp` 设置超时保护。

若整帧输入已经过期、frame 错误或时间倒退，则直接拒绝，不发布新的预测帧。

## 代码做了哪三件事

### 1. 创建 publisher

构造函数中：

```cpp
predictions_publisher_ = create_publisher<
  predictive_nav_msgs::msg::PredictedTrajectoryArray>(
  "/dynamic_obstacles/predictions", rclcpp::QoS(10));
```

`QoS(10)` 表示最多缓存最近 10 帧，适合这个约 10 Hz 的短时数据流。它不是持久化地图；新订阅者应等待下一帧预测，而不是依赖历史轨迹。

### 2. 将内部预测结果转换成 ROS 消息

`make_predicted_trajectory(...)` 做转换：

1. 调用第 06、07 步的 `propagate_cv_positions(...)`。
2. 把每个 `double` 时间偏移转换为 `builtin_interfaces/Duration`。
3. 为每个未来点创建一个 `PoseWithCovariance`。
4. 填入 x/y 位置、2×2 位置协方差、尺寸、ID、置信度。

这里的 `emplace_back()` 是“在 `poses` 容器末尾直接创建一个新 Pose，并返回它的引用”。所以：

```cpp
auto & pose = trajectory->poses.emplace_back();
```

后面的 `pose.pose.position.x = ...` 就是在填写刚添加的那个未来点。

### 3. 发布前再次验证

`is_predicted_trajectory_valid(...)` 会检查：

- `track_id` 不为 0；
- `time_offsets` 和 `poses` 数量相等且不为空；
- 时间偏移严格递增、都大于 0；
- 位置、协方差、尺寸、置信度都不是 NaN/Inf；
- 尺寸为正数。

第 08 步验证的是输入；这里验证的是“转换和计算后的输出”。两层验证有一点重复，但能防止将来改公式或改消息转换代码时，把坏结果直接发布给局部规划。

## 构建与运行

在仿真中，预测节点必须使用 Gazebo 的 `/clock`：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node --ros-args -p use_sim_time:=true
```

另开终端，确认话题和发布者：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic info /dynamic_obstacles/predictions --verbose
```

应至少看到：

```text
Type: predictive_nav_msgs/msg/PredictedTrajectoryArray
Publisher count: 1
```

再观察一帧完整输出：

```bash
ros2 topic echo --once /dynamic_obstacles/predictions
```

输出很长是正常的，因为每条轨迹默认有 10 个未来点，每个点又有 36 个 6×6 协方差元素。重点只看：

```text
header:
  frame_id: odom
trajectories:
- track_id:  ...
  time_offsets:
  - sec: 0
    nanosec: 200000000
  poses:
  - pose:
      position:
        x: ...
        y: ...
```

也可以只观察频率：

```bash
ros2 topic hz /dynamic_obstacles/predictions
```

只要 tracking 正常在约 10 Hz 发布，并且 Track 帧没有被拒绝，预测话题通常也会接近 10 Hz；这不是硬实时承诺。

## 如何排错

| 现象 | 优先检查 |
| --- | --- |
| 找不到话题 | prediction_node 是否仍在运行；是否重新 `source install/setup.bash`。 |
| 所有输入都显示过期 | 仿真启动时是否遗漏 `-p use_sim_time:=true`。 |
| 看到空 `trajectories: []` | 看 prediction 节点的 `track quality gate` 日志，确认拒绝原因。 |
| 有 `CV preview`，却没有发布轨迹 | 看 `prediction-output validation rejected`，这是输出转换后的二次检查。 |
| `frame_id` 不是 `odom` | 回到第 01、02 模块检查 TF 统一和 tracking 输出。 |

## 本步完成标准

- [ ] `colcon build --packages-select predictive_nav_prediction` 成功。
- [ ] `ros2 topic info /dynamic_obstacles/predictions --verbose` 显示类型正确、发布者数量为 1。
- [ ] `ros2 topic echo --once /dynamic_obstacles/predictions` 能看到至少一个 `track_id`、多个 `time_offsets` 和对应的 `poses`。
- [ ] 能解释 `header.stamp + time_offsets[i]` 才是第 i 个预测点的实际时刻。
- [ ] 知道空数组代表“本帧没有可信预测”，不等于节点死掉。
- [ ] 知道本步尚未把预测接入 MPPI 或 `/cmd_vel`。

## 下一步

第 10 步在 RViz 中把每条预测轨迹画出来，避免只靠密集的终端文字判断算法是否合理。
