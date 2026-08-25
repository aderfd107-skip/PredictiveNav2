# 08：发布障碍物候选观测

## 这一步完成了什么

第 07 步已经能在 `scan_info_node` 内部算出很多 `ObstacleCluster`。但“内部变量”只能被当前这个 C++ 文件看见；第 02 模块的跟踪节点无法直接使用它。

本步把每一帧的 cluster 打包成正式 ROS 2 消息，并发布到：

```text
/dynamic_obstacles/clusters
```

因此数据流从：

```text
/scan → scan_info_node → 终端日志
```

变成：

```text
/scan → scan_info_node → /dynamic_obstacles/clusters → （第 02 模块的跟踪节点）
```

这条新 topic 的来源仍然只有 `/scan` 和 TF。它**不读取** Gazebo 动态方块的真值位置或 RViz 真值 Marker。

## 为什么要新建 `predictive_nav_msgs` 包

`predictive_nav_perception` 负责“怎样从激光算出 cluster”。

而“一个 cluster 消息长什么样”是感知、跟踪、预测以后都会共同使用的约定。若把消息定义塞进感知包，后面的跟踪包会反过来依赖感知算法包，结构会越来越乱。

所以新建了一个只放接口的包：

```text
src/predictive_nav_msgs/
├── msg/ObstacleCluster.msg
└── msg/ObstacleClusterArray.msg
```

它不做算法、不运行节点；ROS 2 会根据 `.msg` 文件自动生成 C++ 头文件和命令行可识别的消息类型。

## 这条消息里有什么

一帧激光产生一个 `ObstacleClusterArray`：

```text
header
  stamp:    原始 /scan 的测量时刻
  frame_id: odom

clusters[]
  第 0 个 cluster：中心点、尺寸、点数
  第 1 个 cluster：中心点、尺寸、点数
  ...
```

其中每个 `ObstacleCluster` 的字段是：

| 字段 | 含义 |
| --- | --- |
| `centroid.x`, `centroid.y` | cluster 中心在 `odom` 中的坐标，单位米。 |
| `centroid.z` | 本项目是 2D LiDAR，固定为 0。 |
| `size_x_m`, `size_y_m` | 第 07 步计算的轴对齐包围盒尺寸，单位米。 |
| `point_count` | 构成该 cluster 的有效激光点数。 |

`header` 放在整个数组上，而不是在每一个 cluster 里重复放一遍。原因是：一帧中的所有 cluster 都来自**同一时刻**、**同一坐标系**的 scan。这样消息更简洁，跟踪节点也能明确比较哪两帧。

注意：这个消息没有 `track_id`、速度、置信度或未来轨迹。现在只是观测；这些信息必须由第 02、03 模块的跨帧算法产生，不能在本章凭空填上。

## 代码做了哪些事

### 1. 定义消息

`ObstacleCluster.msg`：定义单个 cluster。

```text
geometry_msgs/Point centroid
float32 size_x_m
float32 size_y_m
uint32 point_count
```

`ObstacleClusterArray.msg`：定义一整帧。

```text
std_msgs/Header header
ObstacleCluster[] clusters
```

`CMakeLists.txt` 中的 `rosidl_generate_interfaces(...)` 是“请 ROS 2 根据这些定义生成代码”的意思。它不是你现在需要手写或背下来的 C++。

### 2. 创建发布者

在 `ScanInfoNode` 创建：

```cpp
cluster_publisher_ = create_publisher<
  predictive_nav_msgs::msg::ObstacleClusterArray>(
  "/dynamic_obstacles/clusters", rclcpp::SensorDataQoS());
```

它的含义可以直接读成：创建一个专门发布 `ObstacleClusterArray` 的出口，名称为 `/dynamic_obstacles/clusters`。

这里继续使用 `SensorDataQoS`：cluster 是从实时 LiDAR 派生出的高频数据；跟踪器更应该拿到最新一帧，而不应该因为追赶过旧帧造成延迟。

### 3. 将内部结构复制到 ROS 消息并发布

每帧聚类后，程序调用：

```cpp
publish_clusters(*message, clustering_result.clusters);
```

这个函数做三件重要的小事：

1. 把输出时间戳复制为输入 scan 的时间戳；
2. 将输出 `frame_id` 明确写为 `odom`，因为 cluster 已经在第 06 步转到了 `odom`；
3. 逐个复制中心、尺寸和点数后调用 `publish(...)`。

最容易犯的错误是仍然复制原始 scan 的 `frame_id=lidar_link`。那样下游会以为 `centroid` 相对 LiDAR，而数值实际已经是 `odom` 坐标，跟踪会立刻出错。因此这里必须用 `tracking_frame_`。

## 构建并检查消息类型

先重新构建。由于这次新增的 `predictive_nav_msgs` 是感知包的依赖，请使用 `--packages-up-to`，它会按正确顺序构建消息包和感知包：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to predictive_nav_perception
source install/setup.bash
```

再用下面命令确认 ROS 2 已经认识这条自定义消息：

```bash
ros2 interface show predictive_nav_msgs/msg/ObstacleClusterArray
```

你应看到 `header`、`clusters`，以及 cluster 内部的 `centroid`、`size_x_m`、`size_y_m`、`point_count`。这一步只检查“接口是否生成成功”，还不需要 Gazebo。

## 在真实仿真中验证 topic

终端 A：保持原来的 Gazebo / Nav2 场景运行。

终端 B：运行感知节点：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

终端 C：订阅一帧输出。发布者使用了传感器的 best-effort QoS，因此命令也要显式使用同样的可靠性设置：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /dynamic_obstacles/clusters --once --qos-reliability best_effort
```

成功时大致会看到：

```yaml
header:
  stamp:
    sec: ...
  frame_id: odom
clusters:
- centroid:
    x: ...
    y: ...
    z: 0.0
  size_x_m: ...
  size_y_m: ...
  point_count: ...
```

具体 cluster 数量和数值会随机器人位置、墙面、家具和动态方块位置变化；不要求与你看到的示例相同。

再检查话题连接信息：

```bash
ros2 topic info /dynamic_obstacles/clusters -v
```

你应看到发布者是 `scan_info_node`，类型是 `predictive_nav_msgs/msg/ObstacleClusterArray`，并能看到 best-effort QoS。

## 这一步的完成标准

- [ ] `colcon build --packages-up-to predictive_nav_perception` 成功，显示 `predictive_nav_msgs` 和 `predictive_nav_perception` 都完成。
- [ ] `ros2 interface show predictive_nav_msgs/msg/ObstacleClusterArray` 显示消息字段。
- [ ] `ros2 topic echo ... --once --qos-reliability best_effort` 收到一帧，且 `header.frame_id` 是 `odom`。
- [ ] `clusters` 中至少有一个 cluster，且包含中心、尺寸、点数。
- [ ] 我能解释：为什么输出需要保留输入 scan 的时间戳，但不能保留 `lidar_link` 这个 frame 名称。

## 本章关联的 ROS 2 与导航知识

**本章必须懂**：ROS 2 的 topic 是节点间的数据契约；`.msg` 文件定义契约，发布者填入消息，订阅者按同一个定义读取。自定义消息要独立放在接口包中，避免跟踪包依赖感知算法实现。QoS 两端必须兼容，否则 topic 名称正确也收不到数据。

**可选扩展**：现在不必理解 DDS 序列化、ROSIDL 生成出的全部文件，或 action/service。第 02 模块只需订阅这一条 cluster topic；第 04 模块才会进一步接触 Nav2 的插件接口。

下一步第 09 步会在 RViz 中画出这些 cluster，并用可视化验证“发布的数据确实位于机器人周围正确的位置”。
