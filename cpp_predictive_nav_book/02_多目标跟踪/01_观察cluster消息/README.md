# 01：观察 cluster 消息

## 这一步做什么，不做什么

这一步**不写任何新 C++ 代码**。第 01 模块已经完成了跟踪器的输入接口，我们先用 ROS 2 命令亲眼确认它的内容。

第 02 模块的跟踪节点将只订阅：

```text
/dynamic_obstacles/clusters
```

它不会订阅：

```text
/scan                                   # 这是感知模块的原始输入
/dynamic_obstacles/cluster_markers     # 这只是 RViz 调试图形
/dynamic_obstacle/ground_truth_marker  # 这是 Gazebo 真值，算法禁止使用
```

这条边界非常重要：跟踪器应该只知道“感知模块观察到了什么”，而不应该知道 Gazebo 里物体的真实答案。

## 先确保第 01 模块正在运行

你应该已经有两个终端：

- **终端 A**：Gazebo / Nav2 / RViz 场景仍在运行；
- **终端 B**：感知节点仍在运行：

```bash
ros2 run predictive_nav_perception scan_info_node
```

如果终端 B 被关闭了，先在新终端重新启动：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

下面所有检查命令都在**第三个终端**执行。每次打开新终端，都先执行：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 检查 1：ROS 2 是否认识这条自定义消息

运行：

```bash
ros2 interface show predictive_nav_msgs/msg/ObstacleClusterArray
```

你会看到类似：

```text
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
ObstacleCluster[] clusters
  geometry_msgs/Point centroid
  float32 size_x_m
  float32 size_y_m
  uint32 point_count
```

先不在意缩进和底层类型名称，只要看懂下面这个结构：

```text
一帧 ObstacleClusterArray
├── header：这一整帧是什么时间、在哪个坐标系
└── clusters[]：这一帧中 0 个、1 个或多个障碍物候选
    ├── 中心点
    ├── x/y 尺寸
    └── 激光点数
```

`clusters[]` 中的 `[]` 表示数组：数量每一帧都可能不同。墙面被看到多少、动态方块是否被遮挡，都会影响它的长度。

## 检查 2：话题是否真的存在，QoS 是否匹配

运行：

```bash
ros2 topic info /dynamic_obstacles/clusters -v
```

正常时，你应看到：

```text
Type: predictive_nav_msgs/msg/ObstacleClusterArray
Publisher count: 1
Node name: scan_info_node
Reliability: BEST_EFFORT
```

具体排版和附加字段可能不同，但这三件事必须正确：

1. 类型是 `predictive_nav_msgs/msg/ObstacleClusterArray`；
2. 发布者是 `scan_info_node`；
3. QoS 中的 Reliability 是 `BEST_EFFORT`。

为什么是 best effort？cluster 是从实时 LiDAR 连续生成的数据。跟踪器宁可少一帧，也不希望为了补旧帧而落后于机器人当前环境。

## 检查 3：看一整帧真实 cluster 数据

运行下面命令：

```bash
ros2 topic echo /dynamic_obstacles/clusters --once \
  --qos-reliability best_effort
```

`--once` 的意思是：只收到一帧就退出，不会让终端持续刷屏。

这里必须写 `--qos-reliability best_effort`。如果订阅者默认要求 reliable，而发布者是 best effort，两端 QoS 不兼容，即使 topic 名字完全正确，也可能什么都收不到。

你会看到类似：

```yaml
header:
  stamp:
    sec: 123
    nanosec: 400000000
  frame_id: odom
clusters:
- centroid:
    x: 2.31
    y: -3.08
    z: 0.0
  size_x_m: 0.42
  size_y_m: 0.18
  point_count: 11
```

数值、cluster 数量与示例不同都正常。请重点检查下面四个位置。

### 1. `header.stamp`

这是**这一帧激光真正测到场景的时刻**，不是你敲命令的时刻。

下一步会连续比较两帧。速度计算必须是：

```text
位置变化 / 两帧 stamp 的差值
```

而不是假设每帧永远间隔 `0.1 s`。

### 2. `header.frame_id: odom`

这说明 `centroid.x/y` 是在局部连续坐标系 `odom` 中表达的。

它不是 `lidar_link`：机器人移动时，静止墙面在 `lidar_link` 中会看似移动；转到 `odom` 后，短时间内才可以合理比较同一物体的连续位置。

若你看到 `frame_id: lidar_link`，先停止，不要进入下一步：这通常意味着运行到了旧版本节点，或忘记 `source install/setup.bash`。

### 3. `clusters:` 和每个 `-`

每一个 `-` 开头的块就是一个 cluster。例如：

```yaml
clusters:
- centroid: ...    # 数组第 0 项
- centroid: ...    # 数组第 1 项
```

**数组第 0 项不是 track #0。** 下一帧中，原来的物体可能变成数组第 2 项，甚至因遮挡暂时消失。第 02 模块就是为了解决这个“没有稳定名字”的问题。

### 4. `point_count` 和尺寸

`point_count` 表示这一帧有多少束有效激光被归进该 cluster；它不是 ID，也不等于“目标置信度”。

`size_x_m`、`size_y_m` 是当前可见点的轴对齐尺寸。LiDAR 只能看见物体朝向雷达的表面，因此它们不是物体真实长宽的保证值；后续可作为 track 的尺寸观测保存和慢慢更新。

## 检查 4：确认它确实持续更新

运行：

```bash
ros2 topic hz /dynamic_obstacles/clusters \
  --use-sim-time \
  --qos-reliability best_effort
```

正常情况下，频率会大致接近 `/scan` 的 10 Hz。它不必每一行精确等于 `10.000`；只要稳定在接近 10 Hz，说明感知节点持续在处理每帧激光并发布结果。

按 `Ctrl + C` 退出。

## 本步最重要的结论

到这里，请用自己的话确认下面这句话：

> `/dynamic_obstacles/clusters` 是在 `odom` 中、带原始测量时间戳的单帧障碍物候选列表；其中没有稳定 ID、速度或未来轨迹。

这就是跟踪器的输入。下一步 `02_创建tracking包骨架` 会新建一个独立 C++ 包来订阅它。

## 常见问题

### `Unknown topic '/dynamic_obstacles/clusters'`

通常是 Gazebo、感知节点，或两者之一没有运行。先确认终端 B 中 `scan_info_node` 没有退出，并且它持续打印 `scan #...` 日志。

### `ros2 topic echo` 一直等待，没有输出

先确认命令中包含：

```bash
--qos-reliability best_effort
```

再运行 `ros2 topic info /dynamic_obstacles/clusters -v`，确认发布者数量大于 0。

### 输出中 `clusters: []`

这代表这帧 topic 正常发布，但当前聚类结果为空；它不是跟踪节点的问题。回到感知节点终端，看同一时刻是否显示 `clusters=0`，并按第 07 步检查聚类阈值和 `tracking_points`。

### RViz 有青色方框，但 echo 看不到消息

青色方框来自另一个 `/dynamic_obstacles/cluster_markers` topic。它能显示只说明 Marker 在发布；跟踪仍必须确认正式的 `/dynamic_obstacles/clusters` topic 能回显。

## 本步完成标准

- [ ] `ros2 interface show predictive_nav_msgs/msg/ObstacleClusterArray` 显示 `header` 和 `clusters[]`。
- [ ] `ros2 topic info /dynamic_obstacles/clusters -v` 显示发布者 `scan_info_node` 和 best-effort QoS。
- [ ] `ros2 topic echo ... --once --qos-reliability best_effort` 收到至少一帧。
- [ ] 输出的 `header.frame_id` 是 `odom`。
- [ ] 我知道 cluster 数组下标不是稳定 ID。
- [ ] `ros2 topic hz ... --use-sim-time --qos-reliability best_effort` 大致接近 10 Hz。

## 本章关联的 ROS 2 知识

**本章必须懂**：topic 是模块间的数据契约；`header.stamp` 与 `header.frame_id` 是数据的一部分，而不是可忽略的附加信息。发布者和订阅者的 QoS 必须兼容。

**可选扩展**：现在不用研究 DDS 的底层传输，也不用理解所有 QoS 参数。你只需知道本项目此处使用 best effort，并能在命令行中让 echo/hz 使用兼容设置。
