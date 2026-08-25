# 09：在 RViz 验证并排查障碍物观测

## 这一步要证明什么

第 08 步已经证明：程序能在命令行发布一条 `ObstacleClusterArray` 消息。

但只看数字还不能回答一个关键问题：**这些中心点和尺寸，是否真的画在激光看到的障碍物位置？**

本步增加一个仅用于调试的 topic：

```text
/dynamic_obstacles/cluster_markers
```

它由 `scan_info_node` 根据同一批 cluster 生成 RViz MarkerArray。它不参与跟踪、预测或控制。

RViz 中将出现三类东西：

| RViz 内容 | 含义 | 数据来源 |
| --- | --- | --- |
| 红色激光点 | 当前原始 LiDAR 回波 | `/scan` |
| 青色半透明方框 + 红色小球 | 你写的 C++ 聚类结果：方框是尺寸，小球是中心点 | `/dynamic_obstacles/cluster_markers` |
| 橙色半透明方框 | Gazebo actor 的真值调试参照 | `/dynamic_obstacle/ground_truth_marker` |

最重要的规则：**只有红色激光点和青色 cluster 来自感知算法链路；橙色真值方框绝不能成为算法输入。** 它只帮助你肉眼判断“动态方块附近是否出现了合理的 cluster”。

## 本次新增的可视化代码

`scan_info_node` 新增了 `publish_cluster_markers(...)`：

```text
ObstacleCluster 列表
  → MarkerArray
  → /dynamic_obstacles/cluster_markers
  → RViz
```

每个 cluster 会生成两个 Marker：

1. `CUBE`：青色半透明方框，中心放在 cluster centroid，`x/y` 尺寸取第 07 步算出的轴对齐包围盒；
2. `SPHERE`：红色小球，表示同一个 cluster 的 centroid。

如果尺寸特别小，显示用方框会把宽高最小限制为 `0.05 m`，避免 RViz 中完全看不见。注意这只影响显示，不会改动 `/dynamic_obstacles/clusters` 中真实发布的 `size_x_m`、`size_y_m`。

### 为什么每帧先发送 `DELETEALL`

假设上一帧有 5 个 cluster，而这一帧只有 3 个。如果只更新编号 0、1、2，旧编号 3、4 的方框会留在 RViz，看上去像“幽灵障碍物”。

因此每次发布 MarkerArray 时先放入一个 `DELETEALL` Marker，再放入当前帧全部方框和中心点。它只清理 `/dynamic_obstacles/cluster_markers` 这个调试 topic 上的旧 Marker，不会清理 Gazebo 的橙色真值 Marker。

### 为什么 Marker 也使用 `odom` 和 scan 时间戳

cluster 的数值已经在第 06 步转换为 `odom`。Marker 如果错误标注为 `lidar_link`，会随着机器人移动而错位；如果标注为 `map`，又会假装已经进行了不存在的转换。

所以 Marker 的：

```text
header.frame_id = odom
header.stamp    = 原始 scan 的测量时间
```

这与第 08 步的 cluster 消息一致。RViz 的 Fixed Frame 虽然是 `map`，但它会通过已有的 `map → odom` TF 把 Marker 显示到地图中。

## 重新构建

这次感知包新增了 `visualization_msgs` 依赖，RViz 配置也新增了 MarkerArray display。请重新构建两个包：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception predictive_nav_bringup
source install/setup.bash
```

## 运行顺序

### 终端 A：启动带动态方块的场景和 RViz

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py \
  enable_dynamic_obstacle:=true
```

等待 Gazebo、RViz 和 Nav2 出现。动态方块开启是为了让你有一个会移动的肉眼参照；即使不开它，静态墙/家具也应出现青色 cluster。

### 终端 B：启动你的感知节点

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

### 在 RViz 中看什么

默认 RViz 配置已经加入 **Scan-Derived Clusters (Debug)**，不需要手动添加 display。

先在 RViz 左侧 Displays 中确认三项均已勾选：

```text
LaserScan
Scan-Derived Clusters (Debug)
Gazebo Ground Truth (Debug)
```

然后将视角拉近机器人和一个动态方块附近，观察：

1. 红色激光点是否打在方块或墙面边缘；
2. 青色方框是否围住一团相邻红点；
3. 红色小球是否处于这团点大致中心；
4. 动态方块经过时，附近的青色方框是否随新的激光观测更新；
5. 橙色真值方框附近是否通常存在青色 cluster。

青色方框不需要和橙色真值框完全重合：LiDAR 只能看到物体朝向传感器的表面，而不是整个实体；聚类又会受到角度、遮挡和阈值影响。你要验证的是位置关系合理，不是像素级重合。

## 三个快速排查命令

如果 RViz 没看到青色方框，依次在新终端执行：

```bash
ros2 topic info /dynamic_obstacles/cluster_markers -v
```

应看到发布者 `scan_info_node`，类型 `visualization_msgs/msg/MarkerArray`。

```bash
ros2 topic echo /dynamic_obstacles/clusters --once --qos-reliability best_effort
```

若它有 `clusters`，说明算法数据已产生；问题更可能在 Marker 或 RViz 显示。

```bash
ros2 run tf2_ros tf2_echo map odom
```

应持续输出变换；若没有，RViz 不能把 `odom` 中的青色方框显示到 Fixed Frame=`map`。

## 常见现象与正确理解

### 有青色方框，但它们覆盖墙面

正常。当前聚类只是在找几何点组，不是在识别“动态物体”。第 02 模块会跨帧维护 ID 和速度，之后才能开始区分动态候选。

### 没有青色方框，但终端显示 `clusters=0`

这是聚类参数或输入问题，不是 RViz 问题。回到第 07 步检查 `tracking_points`、`rejected_clusters`，再试调整 `cluster_distance_threshold_m`。

### 命令行有 cluster，RViz 却没有青色方框

先确认 RViz 左侧 **Scan-Derived Clusters (Debug)** 已勾选，topic 是 `/dynamic_obstacles/cluster_markers`。再检查 Fixed Frame 保持 `map`，并运行 `tf2_echo map odom`。

### 青色方框随着机器人移动而没有固定在墙上

这是严重信号：可能 Marker 或 cluster 的 frame 被误标成了 `lidar_link`。本项目代码应发布 `odom`。用第 08 步的 topic echo 确认 `header.frame_id: odom`。

### 橙色方框和青色方框位置差很大

先不要改算法。按顺序检查：`/scan` 是否真的有点落在该 actor 上、TF 是否稳定、青色方框是否其实对应附近墙面、聚类阈值是否过大。只有排除这些原因后，才讨论坐标或算法问题。

## 本步完成标准

- [ ] 两个包构建成功。
- [ ] RViz 中有 **Scan-Derived Clusters (Debug)** display。
- [ ] 能看到青色方框和红色小球，且它们围住一团红色激光点。
- [ ] 通过 `/dynamic_obstacles/clusters` 回显确认输出 header 的 `frame_id` 是 `odom`。
- [ ] 我能解释：橙色方框是调试真值参照，青色方框才是项目算法产物。

完成第 09 步后，第 01 模块就结束：你的项目已经能把 `/scan` 转成可被下一模块消费、并且经过可视化验证的障碍物候选观测。

## 本章关联的 ROS 2 与导航知识

**本章必须懂**：RViz 只负责显示，不产生算法结果。Marker 的 header 同样必须有正确 frame 和时间；`MarkerArray` 只是把一帧多个可视化对象装在一条 topic 里。看到 Marker 不代表避障已完成，必须始终区分“观测”“跟踪”“预测”和“控制”。

**可选扩展**：不用现在学习 interactive marker、Foxglove 或完整 rosbag 工作流。第 02 模块开始前，只要会用 RViz、`ros2 topic info`、`ros2 topic echo` 和 `tf2_echo` 排查即可。
