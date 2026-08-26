# 03：订阅 cluster 并打印

## 这一小步完成什么

现在 `tracking_node` 已经订阅：

```text
/dynamic_obstacles/clusters
```

每收到十帧，它会打印一次：

```text
这是多少帧 | 这帧在哪个坐标系 | 测量时间 | 有几个 cluster
第一个 cluster 的中心 | 尺寸 | 激光点数
```

这一步的作用只有一个：证明第 01 模块的感知输出，真的能作为第 02 模块的跟踪输入。

现在还**没有**：

- `track_id`；
- 两帧 cluster 的匹配；
- 速度、卡尔曼滤波、协方差；
- `/dynamic_obstacles/tracks` 输出。

不要因为终端已经打印了 cluster，就误以为“已经在跟踪”。此时它只是接收和观察数据。

## 新增代码在做什么

### 1. 引入第 01 模块定义的消息类型

代码新增：

```cpp
#include "predictive_nav_msgs/msg/obstacle_cluster_array.hpp"
```

第 08 步定义的 `.msg` 文件，经过 ROS 2 构建后会生成这个 C++ `.hpp` 头文件。引入它以后，C++ 才知道 `ObstacleClusterArray` 里面有 `header` 和 `clusters`。

### 2. 创建订阅者

构造函数中的核心代码是：

```cpp
cluster_subscription_ = create_subscription<
  predictive_nav_msgs::msg::ObstacleClusterArray>(
  "/dynamic_obstacles/clusters",
  rclcpp::SensorDataQoS(),
  [this](... message) {
    cluster_callback(message);
  });
```

可以按顺序读：

1. 我想接收的消息类型是 `ObstacleClusterArray`；
2. 我要订阅的 topic 名称是 `/dynamic_obstacles/clusters`；
3. 使用 `SensorDataQoS()`，与感知节点发布时的 best-effort QoS 兼容；
4. 收到一帧消息时，调用本类的 `cluster_callback(message)`。

`[this]` 是 C++ lambda 的写法。现在只需要理解：它让这个小回调函数能够调用同一个 `TrackingNode` 中的成员函数；不用自己手写 lambda。

### 3. 回调函数为什么只每十帧打印一次

```cpp
++message_count_;
if (message_count_ % 10U != 0U) {
  return;
}
```

`message_count_` 从 0 开始，每收到一帧加 1。`% 10U` 是“除以 10 的余数”：只有第 10、20、30……帧余数为 0，才继续打印。

感知消息约 10 Hz；如果每帧都打印，终端会很快刷满，反而不容易看问题。这里的 `return` 只跳过**打印**，不会停止订阅。

### 4. 为什么专门检查 `frame=odom`

```cpp
if (message->header.frame_id != "odom") {
  RCLCPP_WARN(...);
}
```

跟踪必须在连续的局部坐标系中比较位置。若中心点还在 `lidar_link`，机器人一动，静止墙也会看似移动。第 01 模块已经把点转换到 `odom`，因此正常日志必须是 `frame=odom`。

目前只是警告，不会停止节点；目的是让错误立刻可见。后面的步骤会把 frame 检查变成更严格的输入验证。

### 5. 为什么只打印第一个 cluster

一帧可能有许多 cluster。现在打印全部会让你看不到最重要的时间和数量信息。

```cpp
const auto & first_cluster = message->clusters.front();
```

意思是：临时引用数组第一个 cluster，用来展示格式。它**不是**“第一个动态障碍物”，也不保证下一帧仍是同一个物体；后面正是要解决这个问题。

## 构建

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

## 运行顺序

你需要三个终端。

### 终端 A：保持仿真场景运行

保持之前的 Gazebo / Nav2 / RViz 场景运行即可。若需要重启：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py \
  enable_dynamic_obstacle:=true
```

### 终端 B：运行第 01 模块的感知节点

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

### 终端 C：运行新的跟踪节点

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_tracking tracking_node
```

## 预期日志怎样看

终端 C 先显示：

```text
Waiting for obstacle-cluster messages on /dynamic_obstacles/clusters.
```

约一秒后，正常情况下会持续出现类似：

```text
cluster frame #10 | frame=odom | stamp=... | clusters=7 |
first_centroid=(..., ...) m | size=(..., ...) m | points=...
```

你应该确认：

- `frame=odom`；
- `clusters` 通常大于 0；
- `stamp` 在持续变大；
- 日志编号从 `#10`、`#20`、`#30` 继续增加。

`first_centroid` 的数值可能随 robot、动态方块、墙面遮挡而变化。不要尝试记住具体数字。

## 常见问题

### 只看到 “Waiting ...”，之后没有新日志

这说明 `tracking_node` 已启动，但没有收到 cluster 消息。依次检查：

1. 终端 B 的 `scan_info_node` 是否仍在运行；
2. 终端 B 是否持续显示 `clusters=...`；
3. 新终端运行：

```bash
ros2 topic info /dynamic_obstacles/clusters -v
```

正常时应同时有发布者 `scan_info_node` 和订阅者 `tracking_node`。

### 出现 “tracking expects odom” 警告

不要进入下一步。把完整警告发给我；它表示输入坐标系错误，会让后续速度估计不可信。

### `clusters=0`

说明消息已经传到了跟踪器，但这一帧没有聚类结果。问题在感知输入或第 07 步参数，不在本步的订阅代码。先查看感知节点同一时刻的日志。

### 终端 C 报找不到 `tracking_node`

通常是忘记构建或忘记执行：

```bash
source install/setup.bash
```

## 本步完成标准

- [ ] `colcon build --packages-select predictive_nav_tracking` 成功。
- [ ] `tracking_node` 显示等待 cluster 的启动日志。
- [ ] 场景和感知节点运行后，日志持续显示 `cluster frame #10`、`#20`……。
- [ ] `frame=odom`，且正常情况下 `clusters` 大于 0。
- [ ] 我知道 `first_centroid` 只是本帧数组的第一项，不是稳定 ID。

## 本章关联的 ROS 2 / C++ 知识

**本章必须懂**：订阅者必须知道消息类型、topic 名称和兼容 QoS；收到消息后 callback 被调用。`ConstSharedPtr` 让回调读取这一帧消息而不复制整份数组。

**可选扩展**：暂时不需要学习多线程 executor、callback group 或 QoS 的所有策略。当前一个订阅、一个回调已经足够进入跟踪算法。

下一步会定义 `Track` 数据结构：它会跨多帧保留状态，和当前只存在于一次 callback 中的 cluster 完全不同。
