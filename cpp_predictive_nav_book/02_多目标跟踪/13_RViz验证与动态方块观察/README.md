# 13：RViz 验证与动态方块观察

## 这一步要解决什么问题

第 12 步已经能在终端或 `ros2 topic echo` 中看到 `/dynamic_obstacles/tracks`，但一大串数字很难判断：

- Track 的位置是否真的贴着对应障碍物？
- 同一个方块运动时，`track_id` 是否稳定？
- 速度方向是不是大致正确？
- 某帧没看到时，Track 是立刻消失，还是进入 `missed` 状态？

所以这一步把正式 Track 数据**画出来**。它不改卡尔曼滤波、不改数据关联、更不控制小车；只是增加一条只供 RViz 调试的输出：

```text
/dynamic_obstacles/clusters
           ↓
    tracking_node
      ├── /dynamic_obstacles/tracks          正式机器接口，给第 03 模块使用
      └── /dynamic_obstacles/track_markers   本步新增，只给 RViz 看
```

`tracks` 才是后续预测模块应订阅的数据。`track_markers` 只是把相同状态画成图，绝不能让后续算法反过来读取 Marker。

## RViz 中三种图形分别表示什么

同时打开三个 display 后，你会看到：

| 画面元素 | Topic / 来源 | 含义 | 能不能作为算法输入 |
|---|---|---|---|
| 青色方框与红色中心点 | `/dynamic_obstacles/cluster_markers` | 本帧 LiDAR 聚类出的原始观测 | 可以，tracking 的输入就是对应的 `clusters` 消息。 |
| 绿色或橙色方框、黄色箭头、`ID ...` 文字 | `/dynamic_obstacles/track_markers` | C++ tracking 节点滤波后的 Track 状态 | 后续应读取正式 `/dynamic_obstacles/tracks`，不读 Marker。 |
| 半透明橙色方框 | `/dynamic_obstacle/ground_truth_marker` | Gazebo 中动态 actor 的真值，仅供人眼对照 | **不可以**。 |

颜色规则：

- **绿色 Track 框**：这一帧关联成功，`missed_frames == 0`；
- **橙色 Track 框**：本帧没有匹配到 cluster，仍在保留预测状态；
- **黄色箭头**：滤波后的速度方向，长度约表示未来 `0.7 s` 的位移，最长限制为 `1.2 m`，防止偶发坏速度把 RViz 撑满；
- **文字**：例如 `ID 7 | v=0.28 m/s | miss=0`。

为了不让墙体和家具边缘的静态 Track 把画面淹没，默认只画速度不小于 `0.10 m/s` 的 **运动候选 Track**。这是 RViz 的显示筛选，**不是**动态目标分类：所有 Track 仍完整存在于 `tracks_`，也仍完整发布到 `/dynamic_obstacles/tracks`。

## 一个容易误解的边界

现在的第 11 步会为**所有满足聚类条件的物体**建立 Track，包括静态墙边、家具边缘和动态方块。

因此你可能看到很多 ID，不要认为“每个 ID 都是动态障碍物”。本模块当前完成的是“多目标连续跟踪”；区分静态/动态目标、清理墙体 Track 是后续动态性分类和风险模块的工作。

观察时优先盯住会移动的橙色 Gazebo 方块附近：它附近的青色 cluster 与 Track 框是否随之移动、ID 是否不频繁变化即可。

## 代码到底新增了什么

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)。

### 1. 新增 MarkerArray 发布器

```cpp
track_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
  "/dynamic_obstacles/track_markers", rclcpp::QoS(10));
```

这里使用普通 `QoS(10)`：Marker 是 RViz 调试数据，宁可短暂排队让画面平稳，也不像传感器流那样必须优先丢旧帧。正式的高频机器接口 `/dynamic_obstacles/tracks` 仍使用 `SensorDataQoS()`。

### 2. 每帧先清掉旧 Marker

```cpp
clear_previous.action = visualization_msgs::msg::Marker::DELETEALL;
```

假设上一帧有 Track #1、#2、#3，这一帧 #3 已删除。如果不清理，RViz 会继续保留旧 #3 的图形，形成“幽灵障碍物”。因此每次根据当前 `tracks_` 重新画完整一组图。

### 3. 一个 Track 画三类 Marker

对每个 `Track`：

```text
LINE_STRIP         用 size_x_m / size_y_m 画二维方框
ARROW              用 vx / vy 画速度方向（接近零时不画箭头）
TEXT_VIEW_FACING   显示稳定 ID、速度大小、missed_frames
```

三种 Marker 使用不同 `namespace`：

```text
tracked_obstacle_box
tracked_obstacle_velocity
tracked_obstacle_label
```

namespace 加上 `track_id` 组成了 RViz 内一个 Marker 的唯一身份。相同 ID 的下一帧 Marker 会更新，而不是无止境堆积。

### 4. 为什么使用 cluster frame 的 header

```cpp
box.header = cluster_frame.header;
```

Track 的位置是刚完成 predict / associate / update 后、对应当前 cluster 测量时间的状态。因此 Marker 的 `frame_id` 和时间必须沿用当前 cluster array 的 header，正常应是 `odom`。不能偷偷用 wall-clock 时间，否则 rosbag 回放或 TF 延迟时可能让 Marker 显示到错误时刻。

## 运行

这一步需要三个节点同时运行。每个新终端都先执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

终端 1：启动场景、地图、Nav2 与 RViz。为了观察移动方块，这里开启动态 actor：

```bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

终端 2：运行 C++ 感知节点，生成 cluster：

```bash
ros2 run predictive_nav_perception scan_info_node
```

终端 3：运行刚更新的跟踪节点：

```bash
ros2 run predictive_nav_tracking tracking_node
```

若你之前已经运行着旧的 `tracking_node`，必须先在它的终端按 `Ctrl+C`，再执行终端 3 的命令；旧进程不包含本步的新 Marker 发布器。

若要回到“完整调试模式”，把可视化阈值设为 0：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args \
  -p track_marker_min_speed_mps:=0.0
```

这样会再次显示包括静态墙体/家具边缘在内的全部 Track；适合排错，不适合录展示视频。

## 在 RViz 中确认什么

重新启动 launch 后，左侧应有三个可勾选项：

```text
Gazebo Ground Truth (Debug)
Scan-Derived Clusters (Debug)
Tracked Obstacles (Debug)     ← 第 13 步新增
```

将它们全部勾选。`Tracked Obstacles (Debug)` 的 Topic 必须是：

```text
/dynamic_obstacles/track_markers
```

如果你的 RViz 是旧窗口、没有这项：点击左下角 **Add** → 选择 `MarkerArray`，将其 Topic 改成上述值；或关闭 RViz 后按本页命令重新启动。

稳定后，请按顺序观察：

1. 青色 cluster 是否在移动方块附近出现；
2. 绿色 Track 框是否覆盖或接近对应 cluster；
3. 方块持续运动数秒时，文字中的 ID 是否大致保持不变；
4. 黄色箭头方向是否与方块运动方向大致一致；
5. 短暂看不到某个 cluster 时，Track 是否先变橙色并显示 `miss > 0`，而非立即消失。

不要求速度数值完全准确。当前关联是欧氏距离贪心，cluster 也会受墙体和遮挡影响；本步验收的是“接口、时间、ID 生命周期和画面一致”，不是宣称已经可靠解决交叉遮挡。

## 出问题时怎样排查

### 没有 `Tracked Obstacles (Debug)`

先确认该 RViz display 的 Topic：

```text
/dynamic_obstacles/track_markers
```

然后检查发布者：

```bash
ros2 topic info /dynamic_obstacles/track_markers
```

预期 `Publisher count: 1`。若是 0，通常是 tracking 节点仍是旧进程、没有 source 新的 `install/setup.bash`，或 tracking 节点没有启动。

### 有 Track 框但都是橙色

这表示 Track 正在存活、却不断匹配失败。先看 tracking 终端的 `track lifecycle` 日志和 `missed`；可能是 `association_gate_m` 太小、cluster 抖动/分裂，或自车/障碍物运动导致相邻帧距离超过 gate。此时记录现象，第 14 步用 rosbag 固定输入后再调参数，不要靠一次画面盲改。

### 方框很多、墙边也有 ID

先确认启动时没有传 `track_marker_min_speed_mps:=0.0`。默认的 `0.10` 会隐藏低速 Track；可临时提高到 `0.15` 或 `0.20` 让画面更简洁，例如：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args \
  -p track_marker_min_speed_mps:=0.15
```

这仍不是动态性分类。tracker 还没有动态/静态标签，任何连续 cluster 都会形成正式 Track；后续应使用速度证据、静态地图或语义信息设计分类规则，而不是为了画面好看删除数据。

### 看见橙色真值框，却没有青色 cluster

真值 Marker 不属于算法输入。先查 `/scan`、感知节点和 `/dynamic_obstacles/clusters`；不要因为 Gazebo 真值框存在就推断 LiDAR 已经检测到它。

## 本章必须懂的 ROS2 / Nav2 知识

- `visualization_msgs/MarkerArray` 是调试可视化协议，不是感知/预测的机器接口；
- Marker 的身份由 `(namespace, id)` 决定；同一对会更新，不同对会共存；
- Marker 的 header 同样需要正确的 frame 与时间；
- `DELETEALL` 是防止障碍物减少时仍残留旧图形的常用调试做法；
- RViz 显示正常不等于算法正确，仍要用正式 topic、日志和第 14 步 rosbag 验证。

## 本步完成的边界

已完成：真实 Track 的 ID、滤波位置、速度、miss 状态在 RViz 可见，并可与 cluster 和 Gazebo 真值做**人眼调试对照**。

尚未完成：动态性分类、交叉时防 ID switch、Mahalanobis/Hungarian 关联、预测轨迹、任何基于 Track 的 Nav2 风险控制。

下一步是 [14_rosbag回放与排错](../14_rosbag回放与排错/README.md)：把今天看到的正常与异常场景录下来，变成以后能重复验证的输入。
