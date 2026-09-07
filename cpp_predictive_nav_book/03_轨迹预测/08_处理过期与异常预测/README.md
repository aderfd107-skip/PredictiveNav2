# 08：处理过期与异常预测

## 本步要解决什么

前面第 06、07 步已经可以从 Track 推算未来位置和不确定性，但“能算”不等于“应该算”。如果输入已经过期、时间倒退、速度离谱，或者轨迹刚出生还不稳定，把它交给后面的风险评估反而可能让小车做出错误避障。

因此本步在预测模块入口增加一个**数据质量闸门（quality gate）**：

```text
/dynamic_obstacles/tracks
        ↓
帧级检查：时间戳、frame
        ↓
逐 Track 检查：年龄、丢失、置信度、状态、速度、协方差
        ↓
只让合格 Track 进入 CV 预测预览
```

本步仍然不发布 `/dynamic_obstacles/predictions`，也不会向 `/cmd_vel` 或 Nav2 写入任何内容。我们先把“拒绝坏数据”的行为观察清楚，第 09 步才发布正式预测结果。

## 什么是“过期”

Track 数组外层有一个共享时间戳：

```text
TrackedObstacleArray.header.stamp
```

它表示这些 Track 对应的是哪个时刻的激光观测。预测节点收到消息时，用 ROS 当前时间减去该时间戳：

```text
age = now - header.stamp
```

默认允许的最大年龄为 `0.5 s`。如果一条“现在才收到”的消息其实是 0.8 秒前生成的，它已经不能代表当前障碍物的位置，必须拒绝。

注意：这里的 `age` 不是 `Track.age`。

| 名称 | 含义 | 单位 |
| --- | --- | --- |
| `message age` | 这整帧消息距现在有多久 | 秒 `s` |
| `track.age` | 该 Track 已连续存在多少帧 | 帧 |
| `missed_frames` | 连续多少帧没有匹配到新 cluster | 帧 |

## 预测模块真正能检查什么

原计划里提到 `dt`，但要严谨一点：`TrackedObstacleArray.msg` 内**没有单独的 `dt` 字段**。上游第 02 模块已经通过相邻 cluster 时间戳检查过 `dt`；预测模块不能凭空再检查一个不存在的字段。

本步检查的是它实际收到、可以验证的内容：

### 帧级检查

- `frame_id` 必须是 `odom`。否则位置、速度与后续导航坐标系的语义不一致。
- 时间戳必须为正数，不能是零或倒退。
- 消息年龄不能超过 `max_track_message_age_s`。
- 时间戳不能明显跑到 ROS 当前时间之后，防止时钟或 rosbag 回放配置错误。
- 新帧时间戳必须比上一次接受的帧更新，避免重复或倒序输入。

### 单条 Track 检查

- 位置、速度、尺寸、置信度和完整 4×4 协方差都必须是有限数，不能出现 `NaN` 或 `Inf`。
- 协方差的四个对角线元素不能为负；负方差在数学上没有意义。
- cluster 的 x/y 尺寸必须大于零。
- `age` 至少达到 3 帧，避免刚出生的一次误聚类马上被预测。
- `missed_frames` 最多为 1 帧，允许一次短暂遮挡，但不长期沿用旧状态。
- `confidence` 至少为 0.5。
- 平面速度不能超过 `3.0 m/s`，防止错误关联产生的“瞬移速度”进入预测。

这些阈值不是永恒真理，而是一版保守的工程默认值。未来应使用 rosbag 和真机数据调参。

## 为什么要同时看 `missed_frames` 和 `confidence`

当前跟踪器中，置信度的简化规则是：

```text
confidence = 1 / (1 + missed_frames)
```

所以默认 `missed_frames <= 1` 与 `confidence >= 0.5` 目前效果接近。两项都保留，是为了让预测模块表达两层不同的语义：

- `missed_frames`：轨迹新鲜度、连续缺观测的次数；
- `confidence`：上游对轨迹质量的汇总分数。

以后你把置信度改成更复杂的规则（例如考虑轨迹年龄、聚类尺寸、关联距离）时，预测模块不必推翻接口。

## 新增参数

| 参数 | 默认值 | 意义 |
| --- | ---: | --- |
| `max_track_message_age_s` | `0.5` | 最大允许消息年龄，单位秒。 |
| `max_future_time_skew_s` | `0.1` | 时间戳最多可领先 ROS 当前时间多少秒。 |
| `min_track_age_frames` | `3` | 最少连续观测帧数。 |
| `max_track_missed_frames` | `1` | 最大连续缺失帧数。 |
| `min_track_confidence` | `0.5` | 最低可接受置信度。 |
| `max_track_speed_mps` | `3.0` | 最大可接受平面速度，单位 `m/s`。 |

在仿真中可以临时把最大速度改为 `1.0 m/s`，观察是否会过滤掉错误匹配产生的高速 Track：

```bash
ros2 run predictive_nav_prediction prediction_node --ros-args \
  -p use_sim_time:=true \
  -p max_track_speed_mps:=1.0
```

这只是调试实验；不要为了“让日志好看”而把阈值调得很低。

## 一个容易错的点：仿真必须使用 `use_sim_time`

Gazebo 的 Track 时间戳使用 `/clock` 中的仿真时间。如果预测节点使用电脑的真实系统时间，那么它会拿“2026 年的秒数”减去“Gazebo 从 0 开始的秒数”，所有输入都会被误判为过期。

在仿真中启动预测节点时使用：

```bash
ros2 run predictive_nav_prediction prediction_node --ros-args -p use_sim_time:=true
```

真机没有仿真时钟时，一般保持默认 `use_sim_time:=false` 即可。

## 看懂新增代码

### `validate_track_frame(...)`

这是“整帧检查”。它先检查 `odom`，再计算：

```cpp
const double age_s = (now - stamp).seconds();
```

若不合格，回调函数立刻 `return`，这一帧中的所有 Track 都不会进入 CV 预测。

### `validate_track(...)`

这是“单条检查”。函数返回：

```cpp
struct TrackValidationResult
{
  bool accepted;
  std::string reason;
};
```

`accepted` 告诉调用者这条 Track 能不能使用；`reason` 保存拒绝理由，方便你排错。这里不是异常崩溃，而是可预期的数据质量问题，因此选择“拒绝并记录”而不是让节点退出。

### `input_watchdog_timer_`

订阅回调只有“新消息到达”才会运行。若 tracking 节点被停止，根本不会再有回调，所以仅在回调内判断过期是不够的。

本步额外创建了一个每 250 ms 执行一次的定时器。它检查上一次接受的 Track 帧是否已经超过 0.5 秒：

```text
track input is stale | ... | holding prediction preview until a fresh frame arrives
```

这叫 watchdog（看门狗）。在本项目中它目前只是报警和停止预览；第 09 步正式发布后，它将成为“不要继续发布旧预测”的安全边界。

若 Gazebo 被暂停，仿真时间 `/clock` 也暂停，消息不会随墙上时间变旧。这是正确行为：仿真世界本身没有继续演化。

## 构建与运行

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node --ros-args -p use_sim_time:=true
```

启动后应先看到：

```text
track-quality gate | max_age=0.50 s | min_age=3 frames | max_missed=1 |
min_confidence=0.50 | max_speed=3.00 m/s
prediction_node started. Step 08 previews only accepted CV predictions and never controls Nav2.
```

正常有输入时，可能看到：

```text
track quality gate | accepted=7 | rejected=2 | first_reason=track has not reached min_track_age_frames
CV preview | id=... | ... | Pdiag=(..., ...) m^2
```

这不是报错：新生 Track 被暂时拒绝是预期行为。

## 如何验证“过期报警”

1. 正常启动仿真、感知、跟踪和预测节点，确认 prediction 节点出现 `CV preview`。
2. 保持 Gazebo 继续运行，只在运行 tracking 节点的终端按 `Ctrl+C`。
3. 等待超过 0.5 秒，prediction 节点应每秒提示一次 `track input is stale`。
4. 重新启动 tracking；收到新的、时间更新的 Track 帧后，报警会停止，CV preview 恢复。

不要通过关闭 Gazebo 来测试这件事，因为 Gazebo 一旦停止，仿真时间也停止，时间戳不会继续变老。

## 本步完成标准

- [ ] 使用 `use_sim_time:=true` 启动后能看到 `track-quality gate` 启动日志。
- [ ] 正常输入时能看到 `CV preview`。
- [ ] 刚出生的 Track 会显示为 `min_track_age_frames` 被拒绝，而不是被当成稳定动态障碍物。
- [ ] 停止 tracking、保持 Gazebo 运行后，能看到 `track input is stale`。
- [ ] 能说清：本步只过滤和预览，尚未发布预测、更没有控制小车。

## 下一步

第 09 步会把每帧所有通过质量闸门的 Track 转为正式 `PredictedTrajectoryArray`，发布到 `/dynamic_obstacles/predictions`。
