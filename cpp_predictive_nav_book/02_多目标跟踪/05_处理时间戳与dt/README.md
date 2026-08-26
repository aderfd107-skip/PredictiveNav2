# 05：处理时间戳与 `dt`

## 这一小步完成什么

跟踪器现在从连续两帧 cluster 消息的 `header.stamp` 计算实际时间间隔：

```text
dt = 当前帧 stamp − 上一帧 stamp
```

它不使用“发布频率大约是 10 Hz，所以 `dt=0.1`”这种假设。

正常时，日志会出现：

```text
dt=0.100 s (valid)
```

由于 Gazebo 的仿真时间、CPU 调度、暂停、重启和 rosbag 回放都可能改变帧间隔，真实 `dt` 可能是 `0.099`、`0.101` 或其他接近值。只要稳定地接近 0.1 秒就是正常的。

本步仍然**不创建 Track，也不计算速度**。它只是把后续速度估计、卡尔曼预测必须依赖的时间基础做正确。

## 为什么不能把 `dt` 写死成 `0.1`

第 01 模块看到 `/scan` 约为 10 Hz，表示平均每秒约 10 帧；它不保证每两帧严格相隔 0.100000 秒。

假设物体真实移动了 `0.05 m`：

```text
真实 dt = 0.08 s  → 速度应约为 0.625 m/s
若错误写死 dt=0.1 → 算成 0.500 m/s
```

速度一开始就错，之后卡尔曼预测的未来位置也会越来越不可信。因此本项目固定原则是：**运动相关计算使用消息的测量时间戳，不使用“我以为的频率”。**

## 新增的时间状态

程序对每一帧都给出一种时间状态：

| 状态 | 含义 | 当前处理方式 |
| --- | --- | --- |
| `first` | 节点刚收到第一帧，没有上一帧可相减 | 保存时间，`dt=0`；不报错。 |
| `valid` | `0 < dt <= max_dt_s` | 这是后续真正可用于速度、预测和更新的帧。 |
| `non_positive` | `dt <= 0`、重复时间或时间倒退 | 发出警告，未来跟踪会拒绝这一帧，并重置时间参考。 |
| `too_large` | `dt > max_dt_s` | 发出警告，未来跟踪会拒绝这一帧并重新开始计时。 |

`first` 只会在节点刚启动后的第一条消息发生一次。由于日志每十帧才打印，你通常看到的第一行仍然会是 `valid`。

## `max_dt_s` 参数是什么

本步新增参数：

```text
max_dt_s = 0.50
```

含义是：超过半秒的帧间隔不再被看作一次正常连续运动。

在 10 Hz 的仿真里，正常间隔约 0.1 秒；0.5 秒已经相当于漏掉了约 4 帧。这个情况常发生在 Gazebo 暂停、节点重启、计算机卡顿或 bag 回放跳转后。若拿这么大的间隔直接做“匀速预测”，物体可能被错误推得很远。

现在的策略是：报警并将当前时间作为新的参考点。后续真正的跟踪循环会跳过这类帧，而不是把它用于卡尔曼状态更新。

## 新增代码怎样工作

### 1. 保存上一帧时间

节点新增两个成员：

```cpp
rclcpp::Time previous_cluster_stamp_{0};
bool has_previous_cluster_stamp_{false};
```

第一行存“上一条 cluster 消息的时间”。第二行解决一个 C++ 初学者很容易遇到的问题：程序刚启动时，变量里的 `0` 不能被误当成一条真实历史消息。

因此，第一帧只做：

```text
previous = current
has_previous = true
返回 first
```

从第二帧开始才真正相减。

### 2. `update_delta_time()`

核心函数接收当前消息时间：

```cpp
const rclcpp::Time current_stamp(message->header.stamp, RCL_ROS_TIME);
const DeltaTimeResult delta_time = update_delta_time(current_stamp);
```

`message->header.stamp` 是 ROS 消息格式的时间；`rclcpp::Time` 是 C++ 节点方便做减法的时间对象。两者表示的是同一个测量时刻。

函数内部执行：

```cpp
dt_s = (current_stamp - previous_cluster_stamp_).seconds();
```

`seconds()` 的结果是 `double`，因此可以得到 `0.100` 这种秒数。

### 3. 为什么每次都更新“上一帧时间”

即使本帧的 `dt` 不合法，代码仍把 `previous_cluster_stamp_` 改成当前时间：

```cpp
previous_cluster_stamp_ = current_stamp;
```

这是一次“重新对时”。否则一条坏时间戳会让后面每一帧都继续与很旧的时间相减，持续产生错误的巨大 `dt`。

### 4. `DeltaTimeResult` 和 `enum class`

```cpp
enum class DeltaTimeStatus { ... };
struct DeltaTimeResult { ... };
```

`enum class` 是 C++ 中“只能从固定几个名字里选一个状态”的写法；这里状态只能是 `first`、`valid`、`non_positive`、`too_large`。

`DeltaTimeResult` 把“状态”和“数值 dt”放在一起返回。这样 callback 不需要猜测 `dt=0` 究竟是第一帧还是时间倒退，日志也更清楚。

你暂时不需要自己手写 enum；只要理解它让程序把不同时间情况明确分开处理。

## 构建与运行

先重新构建：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

保持 Gazebo / Nav2 和 `scan_info_node` 运行，再启动：

```bash
ros2 run predictive_nav_tracking tracking_node
```

正常日志类似：

```text
cluster frame #10 | frame=odom | ... | dt=0.100 s (valid) |
active_tracks=0 | next_track_id=1 | bad_dt(non_positive=0, too_large=0)
```

重点看：

- `dt` 稳定接近 `0.100 s`；
- 状态是 `valid`；
- 两个 `bad_dt` 计数在稳定运行时保持 0；
- `active_tracks=0` 仍是当前预期，因为尚未创建轨迹。

## 只改一个参数，理解它的作用

在停止节点后，可以临时把最大允许间隔改得很小：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args \
  -p max_dt_s:=0.05
```

因为正常 `dt` 约为 0.1 秒，它现在会持续警告 `too_large`，并且 `bad_dt(... too_large=...)` 不断增加。

这不是程序坏了，而是你故意把“允许的正常间隔”设得比实际数据还小。

按 `Ctrl + C` 后，以默认参数重新启动：

```bash
ros2 run predictive_nav_tracking tracking_node
```

此时应重新回到 `dt≈0.100 s (valid)`。

## 常见现象

### 启动后第一次日志不是 `first`

正常。第一帧的 `first` 已经被程序保存，但打印被每十帧节流；第十帧通常已经是 `valid`。

### 偶尔出现一次 `too_large`

如果你刚暂停/恢复 Gazebo、重启了节点或系统出现明显卡顿，偶尔一次可以理解。重点是恢复稳定后计数不应不断增长。

### 一直出现 `non_positive`

这不正常。检查 Gazebo 是否在反复重置仿真时间，或是否有多个发布者向同一 cluster topic 发送不同时间线的数据。不要继续进入速度估计；把完整警告日志发给我。

### 想把 `max_dt_s` 改大来消除警告

不要为了让日志好看而随意增大。先判断是否确实发生了暂停/重置；这个阈值是保护后续速度和卡尔曼预测的安全边界。

## 本步完成标准

- [ ] 构建成功。
- [ ] 正常运行时日志显示 `dt` 接近 `0.100 s (valid)`。
- [ ] 稳定运行时 `bad_dt(non_positive=0, too_large=0)` 不增加。
- [ ] 我知道第一帧不能计算 dt，且不能将频率近似值写死为真实 dt。
- [ ] 我用 `max_dt_s:=0.05` 观察过 `too_large` 警告，再用默认参数恢复正常。

## 本章关联的 ROS 2 / C++ 知识

**本章必须懂**：`header.stamp` 是数据测量时间；`rclcpp::Time` 能在 C++ 中安全做时间差。ROS 参数让你无需改代码就能调整阈值。`enum class` 用于明确表示有限种状态。

**可选扩展**：现在不需要研究 ROS clock 的全部实现或时间同步理论。下一步只会使用你已经验证过的 `dt`，以最直观的两帧相减方式观察单目标的朴素速度。
