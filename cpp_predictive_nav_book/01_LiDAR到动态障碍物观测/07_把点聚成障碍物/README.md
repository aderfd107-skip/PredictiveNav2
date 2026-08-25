# 07：把散乱的点聚成障碍物候选

## 这一步到底在做什么

第 06 步结束后，一帧激光已经变成许多位于 `odom` 中的二维点：

```text
719 个点：(-1.2, 0.4)、(-1.2, 0.5)、(-1.2, 0.6)、...
```

但后续跟踪器不能拿着 719 个点逐个分配 ID、估计速度。它需要的单位是“一个物体的观测”，例如：

```text
cluster 1：一组属于某面墙的相邻点
cluster 2：一组属于桌子的相邻点
cluster 3：一组属于动态方块的相邻点
```

**聚类（clustering）**就是把空间上连在一起、距离足够近的点归为一组。每一组称为一个 `cluster`，本步会为它计算：

```text
点数、中心点（centroid）、x 方向尺寸、y 方向尺寸
```

## 一个很重要的边界

这一步的输出还不是“动态障碍物列表”。

仅靠一帧激光，我们只能知道“这里有一团点”，不能知道它是静止墙、家具，还是正在走动的方块。墙、家具和动态方块都会形成 cluster。

真正判断一个 cluster 是否在运动，要在后面比较连续多帧、保持 ID、估计速度后才能完成。这正是多目标跟踪模块的工作。

因此，本步的准确表述是：**将一帧激光点分成障碍物候选观测。**

## 先用一张小图理解

```text
散乱点：                         聚类后：

• • • • •        • • •            [ cluster A ]   [ cluster B ]
• • • • •        • • •     →      中心和尺寸        中心和尺寸
• • • • •

点与点之间近：属于同一组
两团点之间远：分成不同组
```

在真实 LiDAR 中，每个点本来按激光转动顺序排列。我们不做昂贵的“任意点和任意点两两比较”，而是依次比较相邻激光束的点：

```text
第 i 束点 与 第 i+1 束点 足够近 → 同一个候选 cluster
距离超过阈值 或 中间有无效光束 → 结束当前 cluster，开始新 cluster
```

这是适合 2D LaserScan 的**顺序欧氏聚类**。距离使用二维欧氏距离：

\[
d=\sqrt{(x_2-x_1)^2+(y_2-y_1)^2}
\]

代码中的 `std::hypot(dx, dy)` 就是在计算这个距离。

## 本步已经加入真实项目的内容

`scan_info_node.cpp` 已新增：

```text
TrackingPoint 列表
  → cluster_tracking_points()
  → ClusteringResult
  → ObstacleCluster 列表
```

`ObstacleCluster` 目前只保存摘要，不保存所有原始点：

```cpp
struct ObstacleCluster
{
  std::size_t point_count;
  double centroid_x_m;
  double centroid_y_m;
  double size_x_m;
  double size_y_m;
};
```

这已经足以让后续消息发布、数据关联与卡尔曼跟踪使用；调试原始点仍可从前面的 `TrackingPoint` 列表得到。

## 本步的三个参数

| 参数 | 默认值 | 意义 |
| --- | --- | --- |
| `cluster_distance_threshold_m` | `0.18` | 相邻两束点相距不超过多少米时，认为属于同一组。 |
| `min_cluster_points` | `3` | 少于 3 个点的候选组通常是噪声，丢弃。 |
| `max_cluster_points` | `100` | 超过 100 个点的候选组通常是长墙面或过大的结构，暂时不作为可跟踪物体。 |

这些都是第一版经验参数，不是物理定律。之后会根据 Gazebo 和真机实验调节；现在先保留在 ROS 参数中，避免将数字写死在算法里。

## 构建与运行

在新终端中逐行复制：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

另一个终端必须保持 Gazebo / Nav2 场景运行。

你应看到类似：

```text
first cluster | points=... | centroid_odom=(..., ...) m | size=(..., ...) m
scan #... | ... | tracking_points=719 | ... | clusters=... |
rejected_clusters(small=..., large=...)
```

只要 `clusters` 大于 0，说明点已经被分组。数量随机器人姿态、视野和动态方块位置变化是正常的。

## 怎样读懂核心代码

### 1. 先看两个点是否能连在一起

```cpp
bool are_neighbouring_scan_points(
  const TrackingPoint & previous,
  const TrackingPoint & current) const
```

它有两道条件：

1. `current.beam_index` 必须刚好等于 `previous.beam_index + 1`；
2. 两个点的欧氏距离必须不大于 `cluster_distance_threshold_m_`。

为什么还要检查光束下标？第 04 步可能丢弃 NaN 或超量程数据。假设第 100 束无效，而第 99、101 束恰好在空间上靠近；它们不应被不加判断地跨过缺口连接，因为中间真实场景信息已经缺失。

### 2. 形成一个候选组

`cluster_tracking_points()` 从左到右遍历所有点。`candidate_points` 可以理解为“正在收集、尚未封口的这一团点”。

遇到一个不相邻的新点时：

```cpp
append_cluster_if_valid(candidate_points, result);
candidate_points.clear();
```

第一句结束旧候选组、检查它是否有资格成为 cluster；第二句清空列表，准备收集新的一组。

循环结束后还要再调用一次 `append_cluster_if_valid(...)`，否则最后一团点没有遇到“分隔点”，会被遗漏。这是写顺序算法时非常常见的边界条件。

### 3. 为什么小组和大组都要丢弃

`append_cluster_if_valid()` 会分别统计：

```text
rejected_too_small：点太少，常是噪声或单束偶然回波
rejected_too_large：点太多，常是连续墙面或太大的静态结构
```

现在不要把“被丢弃”理解为“数据没有价值”。它只是说：第一版动态物体跟踪器暂不处理这些组。后续还可以单独利用静态地图、尺寸规则或语义识别处理它们。

### 4. 中心和尺寸怎样计算

对于一个有 N 个点的 cluster：

\[
center_x=\frac{1}{N}\sum_{i=1}^{N}x_i,\qquad
center_y=\frac{1}{N}\sum_{i=1}^{N}y_i
\]

这就是 centroid（质心/中心点）。

尺寸暂时使用轴对齐包围盒：

```text
size_x = max(x) - min(x)
size_y = max(y) - min(y)
```

它很容易理解，也足够给后续风险半径一个初始尺寸估计。它不是精确物体长宽；物体旋转时会产生较大包围盒，这是以后可以改进的地方。

## 只改参数，观察算法如何变化

先按 Ctrl + C 停止节点，再启动一个“阈值更小”的版本：

```bash
ros2 run predictive_nav_perception scan_info_node --ros-args \
  -p cluster_distance_threshold_m:=0.08
```

预期现象：相邻点更难连接，`clusters` 往往更多，`rejected_clusters(small=...)` 也可能增加。

再试阈值更大：

```bash
ros2 run predictive_nav_perception scan_info_node --ros-args \
  -p cluster_distance_threshold_m:=0.35
```

预期现象：更多点被连成同一组，`clusters` 往往减少，可能有更多 `large` 组被丢弃。

这就是参数调试：不是死记 0.18，而是理解“阈值变大/变小会让点如何合并或拆分”。

## 常见问题

### `clusters=0`

先确认 `tracking_points` 大于 0。若它是 0，先回到第 06 步检查 TF。

若 `tracking_points` 大于 0，但 `clusters=0`，查看 `rejected_clusters`：

- `small` 很多：可临时把 `min_cluster_points` 改为 2，确认是否是阈值过严；
- `large` 很多：可临时增大 `max_cluster_points`，确认是否大量点连成了墙；
- 两者都不多但仍为 0：把整行日志发给我，不要直接删掉所有筛选。

### cluster 数量每帧有一点变化

这是正常的。激光噪声、机器人移动、边缘点时有时无都会让点数和边界略变。第 09 步的多帧跟踪正是用来稳定这种变化的。

### 为什么不直接使用 DBSCAN 或匈牙利算法

DBSCAN 是更通用的聚类方法；匈牙利算法是跨帧分配 ID 的方法，根本不是聚类。本项目第一版的目标是一个可解释、易调试的 2D LaserScan 顺序欧氏聚类。只有实验显示它不能满足需求时，才增加更复杂的方法作为对照。

## 本步完成标准

- [ ] 重新构建成功。
- [ ] 日志中 `tracking_points` 大于 0、`clusters` 大于 0。
- [ ] 我知道 cluster 是几何点组，不等于“已经识别出动态障碍物”。
- [ ] 我理解小阈值让点更容易拆分，大阈值让点更容易合并。
- [ ] 我用一次参数命令观察过阈值变化。

下一步会定义并发布正式的 cluster 消息，让后面的独立跟踪节点可以订阅这些障碍物候选观测。

## 本章关联的 ROS 2 与导航知识

**本章必须懂**：这一步运行在 ROS 2 节点中，但“聚类”本身是普通算法，不是 ROS 2 魔法。ROS 2 只负责把每帧 `/scan` 送进回调；聚类代码把 `odom` 中相邻的点变成一个 cluster。一个 cluster 只是障碍物**候选观测**，不能仅凭这一帧判断它是静态物体还是动态物体。

**可选扩展**：暂时不必实现 DBSCAN、PCL 或 costmap。等第 02 模块做跨帧关联时，再比较“最近邻”和匈牙利算法；等第 04 模块接入 Nav2 时，再讨论 cluster 怎样影响局部 costmap 和 MPPI 风险评分。
