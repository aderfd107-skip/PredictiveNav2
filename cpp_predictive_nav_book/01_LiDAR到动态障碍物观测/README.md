# 第一模块：从 LiDAR 到动态障碍物观测

## 这就是项目的下一步

你现有的仿真已经在 `/scan` 发布 2D LiDAR 数据。它是一长串“每个角度测到多远”的数；它还不是“第 3 个动态障碍物在什么位置”。

本模块会新增真正的 ROS 2 C++ 包 `predictive_nav_perception`，让它最终发布 `/dynamic_obstacles/clusters`。每个 cluster 表示一团相邻激光点组成的障碍物观测，包含中心位置、尺寸和点数。

## 这不是一次性写完

先完成一个能订阅 `/scan` 并打印基本信息的最小节点，再依次过滤坏数据、转成二维点、转换到 `odom`、做聚类、发布结果并在 RViz 验证。每一步都能单独运行，且只引入少量新 C++。

## 小步骤顺序

1. [00_这一模块要做什么](00_这一模块要做什么)：理解最终输入、输出和数据流。
2. `01_观察现有scan数据`：先在不写代码的情况下确认真实话题内容。
3. `02_创建真实C++包`：创建工程骨架；理解它每个文件的职责。
4. `03_订阅scan并打印`：第一个项目 C++ 节点。
5. `04_筛选有效激光距离`：过滤 NaN、无穷大和不可信读数。
6. `05_距离变成二维点`：极坐标转 `(x, y)`。
7. `06_统一到odom坐标系`：处理 TF 和时间戳。
8. `07_把点聚成障碍物`：得到 cluster。
9. `08_发布障碍物观测`：定义和发布项目消息。
10. `09_RViz验证与排错`：证明功能真的使用 `/scan`，而不是 Gazebo 真值。

现在先不要打开后面的目录。我们从第 00 步开始，一次只处理一件事。

## 本模块的 ROS 2 知识怎样补

不需要脱离项目单独学。每个小步骤都已经在 [ROS2与Nav2补充路线](../00_从这里开始/ROS2与Nav2补充路线.md) 中列出了“本章必须懂”和“可选扩展”。当前只看必需项；可选扩展等本章跑通、或者你确实遇到相关问题时再看。

## 模块完成总结：你已经做成了什么

第一模块已经完成。你现在不是“读取了一条 `/scan`”，而是已经把原始 LiDAR 数据加工成了下一模块能够直接使用的、带时间和坐标语义的障碍物候选观测。

完整数据流如下：

```text
Gazebo 2D LiDAR
  ↓ 发布
/scan（LaserScan：每个角度测到的距离）
  ↓ predictive_nav_perception / scan_info_node
  ├─ 过滤 NaN、Inf 和超出有效距离的读数
  ├─ 距离 + 角度 → lidar_link 中的二维点
  ├─ 按 scan 原始时间戳查询 TF：odom ← lidar_link
  ├─ 得到 odom 中的二维跟踪点
  └─ 顺序欧氏聚类：点组 → cluster（中心、尺寸、点数）
  ↓ 发布
/dynamic_obstacles/clusters（真正给后续算法使用）
  ↓ 仅调试可视化
/dynamic_obstacles/cluster_markers → RViz 青色方框与红色中心点
```

## 每一步实际解决了什么

| 步骤 | 解决的问题 | 最终留下的能力 |
| --- | --- | --- |
| 01 | 不确定仿真实际发布什么 | 确认 `/scan`、`LaserScan`、仿真时间与约 10 Hz 数据流。 |
| 02～03 | 没有自己的 C++ 感知程序 | 创建并运行 `predictive_nav_perception/scan_info_node`。 |
| 04 | 原始 scan 里有无效/不可信数据 | 丢弃 NaN、Inf 和量程外距离，并保留原始 beam 下标。 |
| 05 | 距离不是空间位置 | 使用角度把每束激光转换为 `lidar_link` 中的 `(x, y)`。 |
| 06 | 机器人移动会让静止物体在雷达坐标中看似移动 | 用 scan 自身时间戳将点转到连续的 `odom` 坐标系。 |
| 07 | 719 个点无法直接逐个跟踪 | 把相邻点归为 cluster，计算中心、尺寸和点数。 |
| 08 | cluster 只存在于当前 C++ 文件内部 | 定义项目自有消息并发布 `/dynamic_obstacles/clusters`。 |
| 09 | 只看终端数字无法判断空间是否正确 | 在 RViz 中看到 scan 派生的青色方框和红色中心点。 |

## 你已经验证过什么

- `/scan` 在使用仿真时间时稳定接近 10 Hz；
- 有效激光点能从 `lidar_link` 正确转换到 `odom`；
- 启动初期少量 TF Buffer 失败不会持续增长，稳定后 `tracking_points` 正常产生；
- cluster 消息能从 `/dynamic_obstacles/clusters` 回显，header 的 `frame_id` 是 `odom`；
- RViz 中的青色方框/红色中心点已经出现，且对应 `/scan` 中的一团激光点。

这意味着第 01 模块不是只有“代码能编译”，而是已经完成了**输入、算法、ROS 接口、可视化**四层验证。

## 现在的边界：它还不等于动态障碍物跟踪

虽然 topic 名称包含 `dynamic_obstacles`，当前发布的仍是“障碍物候选观测”。一面墙、一张桌子和一个移动方块都可能形成 cluster。

目前系统还**没有**：

- 同一物体跨帧的稳定 ID；
- 速度或运动方向；
- “静态/动态”判断；
- 未来位置和不确定性；
- 对 Nav2 控制器的风险评分或避障决策。

因此，RViz 中的橙色 Gazebo 方框只能用于肉眼对照，绝不能作为算法输入；第 02 模块也只订阅 `/dynamic_obstacles/clusters`，不订阅任何真值 Marker。

## 这章中自然学到的 ROS 2 / C++

你不必背语法定义，但已经在真实项目中用到了：

- ROS 2 node、topic、订阅回调、发布者、参数和 `SensorDataQoS`；
- `LaserScan` 的 `ranges`、`header.frame_id`、`header.stamp`；
- TF2、`map → odom → base_footprint → lidar_link` 坐标链，以及为何用 scan 原始时间查询 TF；
- C++ 的 `class`、成员函数、`struct`、`std::vector`、循环、条件判断、异常处理和自定义消息对象；
- `package.xml`、`CMakeLists.txt`、`colcon build` 与 ROS 2 接口包；
- RViz、`ros2 topic echo`、`ros2 topic info`、`tf2_echo` 的基本排错方式。

这些不是脱离项目的知识点；它们就是你当前感知节点为什么能工作的组成部分。

## 关键源码位置

```text
src/predictive_nav_perception/src/scan_info_node.cpp
    本模块的完整 C++ 感知、聚类、发布与 Marker 可视化代码

src/predictive_nav_msgs/msg/ObstacleCluster.msg
src/predictive_nav_msgs/msg/ObstacleClusterArray.msg
    感知模块输出给跟踪模块的正式数据契约

src/predictive_nav_bringup/rviz/nav_baseline.rviz
    RViz 中 Scan-Derived Clusters (Debug) 的显示配置
```

## 下一模块从哪里接上

第 02 模块只从这一条正式接口开始：

```text
/dynamic_obstacles/clusters
  header.stamp / header.frame_id=odom
  clusters[]：centroid、size_x_m、size_y_m、point_count
```

它要回答的新问题是：**这一帧中心在 `(x, y)` 的 cluster，和上一帧的哪一个 cluster 是同一个物体？**

一旦能持续维护同一个 ID，才能根据位置随时间的变化估计速度，并为第 03 模块的轨迹预测准备数据。
