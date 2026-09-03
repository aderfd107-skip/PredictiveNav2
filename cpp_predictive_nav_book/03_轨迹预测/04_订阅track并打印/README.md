# 04：订阅 Track 并打印

> 本步目标：让 `prediction_node` 真正收到第二模块发布的 `/dynamic_obstacles/tracks`，但只观察和打印，**还不计算任何未来位置，也不发布 `/dynamic_obstacles/predictions`**。

## 1. 为什么预测模块的输入是 Track？

第 01 模块的 cluster 只代表“这一次激光看到了这里有一团点”。它没有稳定 ID，速度也不可信，不能直接作为未来预测的起点。

第 02 模块已经把多帧 cluster 处理成 Track：

```text
LaserScan → cluster → Track
                     ├─ track_id：这是哪个目标
                     ├─ position：现在在哪里
                     ├─ velocity：估计的速度
                     ├─ covariance：当前位置/速度有多不确定
                     └─ header.stamp：这个“现在”究竟是什么测量时刻
```

所以第三模块的正确输入只能是：

```text
/dynamic_obstacles/tracks
```

而不是 `/scan`、`/clusters`、Gazebo 真值或 RViz Marker。

## 2. 本步发生的数据流

```text
tracking_node
    │ 发布 predictive_nav_msgs/msg/TrackedObstacleArray
    │ 话题：/dynamic_obstacles/tracks
    ▼
prediction_node
    │ 订阅、每秒打印一次摘要
    ▼
终端（供你观察）
```

此时 `prediction_node` 是订阅者，不是发布者。它绝不会向小车发送速度命令。

## 3. 代码新增了什么？

### 3.1 新依赖 `predictive_nav_msgs`

`TrackedObstacleArray` 是我们自己定义的消息，文件在：

```text
src/predictive_nav_msgs/msg/TrackedObstacleArray.msg
```

因此预测包必须在两个地方声明依赖：

```text
package.xml      # ROS 2 包层面的依赖声明
CMakeLists.txt   # 编译器如何找到消息头文件
```

二者缺一个，都可能导致编译时找不到：

```cpp
#include "predictive_nav_msgs/msg/tracked_obstacle_array.hpp"
```

### 3.2 创建订阅者

核心代码是：

```cpp
tracks_subscription_ = create_subscription<
  predictive_nav_msgs::msg::TrackedObstacleArray>(
  "/dynamic_obstacles/tracks",
  rclcpp::SensorDataQoS(),
  [this](predictive_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr message) {
    tracks_callback(message);
  });
```

按顺序理解：

| 片段 | 意义 |
| --- | --- |
| `create_subscription<...>` | 创建“接收某种 ROS 消息”的对象 |
| `TrackedObstacleArray` | 接收的消息类型，必须与发布者完全一致 |
| `"/dynamic_obstacles/tracks"` | 订阅的话题名称 |
| `SensorDataQoS()` | 使用与 tracking 发布者兼容的传感器数据 QoS |
| `lambda` | 每收到一帧消息，就调用一次的回调函数 |

`ConstSharedPtr` 可以先拆成两部分理解：

- `Const`：本节点只读取这帧 Track，不允许修改它；
- `SharedPtr`：ROS 2 用智能指针管理消息内存，我们不需要手动 `free`。

这与 C 语言中“回调拿到一个只读数据指针”很接近，只是 C++ 用类型系统和智能指针减少内存错误。

### 3.3 为什么要“节流打印”？

`/tracks` 约 10 Hz。如果每一帧、每条 Track 都完整打印，终端很快被墙体、桌子等静态 Track 填满，真正的动态目标反而看不见。

因此代码使用：

```cpp
RCLCPP_INFO_THROTTLE(..., 1000, ...)
```

最后的 `1000` 表示同一条日志最多每 **1000 ms = 1 s** 打印一次。每次只展示数组中前 3 条 Track；这仅影响人眼调试输出，完整 ROS 消息没有删掉、算法也没有漏算任何目标。

## 4. 你将在终端看到什么？

示例：

```text
[INFO] [prediction_node]: track frame | stamp=123.400 s | frame=odom | track_count=8
[INFO] [prediction_node]:   track[0] | id=7 | pos=(2.31, -0.84) m |
vel=(0.28, 0.01) m/s | P_xy=[[0.0225, 0.0000], [0.0000, 0.0225]] m^2 |
size=(0.35, 0.42) m | age=35 | miss=0 | confidence=1.00
[INFO] [prediction_node]:   ... 5 additional track(s) omitted from this debug log.
```

重点逐项看：

| 输出 | 你应理解成什么 |
| --- | --- |
| `stamp` | 这帧 Track 对应的激光测量时间，不是终端打印的电脑时间 |
| `frame=odom` | 所有 position、velocity 都在 odom 坐标系下表示 |
| `track_count` | 当前存在几条活跃 Track，包含静态墙体/桌子是正常的 |
| `id` | 目标稳定编号；不要把 `track[0]` 数组下标误认为 ID |
| `pos=(x,y) m` | 当前估计位置，单位 m |
| `vel=(vx,vy) m/s` | 当前估计速度，单位 m/s |
| `P_xy` | x/y 位置协方差 2×2 摘要，单位 m²；数值越大通常代表越不确定 |
| `age` | 这个 Track 已存活的帧数 |
| `miss` | 最近连续多少帧没匹配上观测；0 表示本帧看到了 |
| `confidence` | 目前是 Track 新鲜度启发式值，不是精确概率 |

本步不用尝试判断每个静态墙面 Track 是否“合理”；你只需要确认消息连通、时间和 frame 语义正确。后续动态风险模块会避免把“静态地图已有的墙”与“真正移动风险”混淆。

## 5. 如何运行

先确保第二模块的运行链路已经存在。通常需要三个已运行终端：

```bash
# 终端 A：仿真与 Nav2
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

```bash
# 终端 B：感知（cluster）
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

```bash
# 终端 C：多目标跟踪
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_tracking tracking_node
```

然后在新的终端 D 构建并运行 prediction：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node
```

## 6. 检查订阅是否真的建立

保持 `prediction_node` 运行，另开终端执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 node info /prediction_node
ros2 topic info /dynamic_obstacles/tracks --verbose
```

正常情况下，`ros2 node info /prediction_node` 应在 `Subscribers` 中列出：

```text
/dynamic_obstacles/tracks: predictive_nav_msgs/msg/TrackedObstacleArray
```

而 `ros2 topic info` 应显示 tracking 的 Publisher 和 prediction 的 Subscriber。

## 7. 常见现象与排错

### 节点启动了，但没有 `track frame` 日志

依次检查：

```bash
ros2 topic info /dynamic_obstacles/tracks
ros2 topic echo --once /dynamic_obstacles/tracks
ros2 node list
```

- Publisher count 为 `0`：tracking 节点没有运行或已经崩溃；
- 能 echo 但 prediction 没打印：确认重新构建后在**同一个终端**执行了 `source install/setup.bash`；
- 看到空数组 `obstacles: []`：订阅本身没问题，只是本帧暂时没有活跃 Track；
- 看到 `frame` 不是 `odom`：先停下，不要往后写预测，先检查第 01/02 模块的 TF 与输出约定。

### 为什么只显示三条 Track？

这是调试日志故意的上限，不是算法筛选。完整消息依然可以用：

```bash
ros2 topic echo --once /dynamic_obstacles/tracks
```

查看。

## 8. 本步完成标准

- [ ] `colcon build --packages-select predictive_nav_prediction` 成功；
- [ ] `prediction_node` 启动后每秒打印一帧 `track frame`；
- [ ] 输出 `frame=odom`；
- [ ] 能从一条日志指出 ID、位置、速度的 x/y 分量和单位；
- [ ] 明白本节点目前只读 Track，没有发布 prediction、更没有控制小车。

## 本章必须懂的 ROS 2 / C++ 知识

- 发布者和订阅者的话题名、消息类型、QoS 必须兼容，数据才会流动。
- ROS 2 订阅通过“回调函数”处理每一帧异步到达的消息。
- `ConstSharedPtr` 表示只读、自动管理生命周期的消息指针。
- 节流日志是工程调试手段：保留足够证据，同时避免高频输出掩盖问题。

## 下一步

[05_确定预测时间轴](../05_确定预测时间轴/README.md)：输入已确认后，先定义预测究竟看未来多久、每隔多久取一个未来点，再写 CV 传播公式。
