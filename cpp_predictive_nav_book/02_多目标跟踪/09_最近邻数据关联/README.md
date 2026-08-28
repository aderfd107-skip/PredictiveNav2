# 09：最近邻数据关联

## 先用一句人话说明

上一帧你看到一个方块在 `(2.25, -2.80)`，并预测它现在应该在 `(2.25, -2.77)`。当前 LiDAR 聚类却给了许多 cluster：墙、桌角、方块的一部分……程序怎样判断“哪一个才是刚才那个方块”？

这一章的答案是最简单、最容易检查的方法：**先从预测位置出发，找当前所有 cluster 中最近的一个；但只有它足够近，才承认它们是同一个目标。**

这个过程叫做**数据关联（data association）**。它不是计算速度，也不是更新 Kalman Filter；它只是决定“这一帧的哪条测量，可以交给哪条已有轨迹”。

## 三个新名词

| 名词 | 白话含义 |
|---|---|
| 预测位置 | 已有轨迹根据上一时刻位置、速度和 `dt` 推测出的当前位置 |
| 最近邻（nearest neighbor） | 在所有候选 cluster 中，选与预测位置二维距离最小的那个 |
| 门限 / gate | 最大允许匹配距离；最近也不一定是同一个，太远就拒绝 |

本步计算的距离是：

```text
distance = sqrt((cluster_x - predicted_x)² +
                (cluster_y - predicted_y)²)
```

如果 `distance <= association_gate_m`，结果为匹配；否则为 `rejected_by_gate`。

## 为什么这比第 06 步好

第 06 步每帧只看“离固定参考位置最近的 cluster”。方块一旦离开参考点、墙体 cluster 靠近参考点，程序就可能突然换目标，于是产生 `4 m/s`、`13 m/s` 的离谱速度。

现在参考点不再固定在地图上，而是来自上一步的**运动预测**。如果目标按原方向移动，预测点也会移动；错误 cluster 若离预测点很远，就会被 gate 拒绝。

不过要诚实说明：本步还只是在一个教学状态上验证规则。它尚未为多个真正的 `Track` 逐一匹配，也尚未更新预测状态。第 10、11 步才会把这些零件组装成完整多目标跟踪循环。

## 写入的代码

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)

新增 `DebugAssociationResult`，它保存：

- 是否已经有 CV 预测；
- 最近 cluster 的指针和本帧数组下标；
- 预测点到该 cluster 的距离；
- 是否通过 gate。

函数 `associate_debug_prediction()` 会遍历当前帧的所有 cluster，找最短距离；`log_debug_nearest_neighbor_association()` 则把判断结果打印出来。

当前的 `cluster_index` 只是这一帧数组中的位置，**不是稳定 ID**。下标可能每一帧变化；真正稳定 ID 会在第 11 步创建真实 Track 时出现。

## 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `association_gate_m` | `0.40` | 预测位置到 cluster 中心的最大可接受距离（米） |

`0.40 m` 不是神奇数字：它需要覆盖 LiDAR 聚类中心误差和短时间预测误差，但又不能大到把相邻墙体或另一个目标误认为同一物体。后续会靠 rosbag 和实验来调参，而不是永远相信默认值。

## 构建与运行

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

继续沿用第 06 步的三个终端：

```bash
# 终端 1：动态仿真
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true

# 终端 2：感知/聚类
ros2 run predictive_nav_perception scan_info_node

# 终端 3：跟踪实验
ros2 run predictive_nav_tracking tracking_node
```

当 CV 状态已经初始化后，会每十帧看到类似：

```text
nearest-neighbor association (debug only) |
predicted=(2.25, -2.77) m |
matched_cluster_index=3 |
centroid=(2.27, -2.80) m |
distance=0.04 m | gate=0.40 m
```

这表示：当前第 3 个 cluster 离预测位置只有 0.04 米，小于 0.40 米，因此暂时认为它就是同一个目标。

另一种正常输出是：

```text
result=rejected_by_gate
```

它不表示程序崩了，而是说“当前所有 cluster 都离预测点太远，不能诚实地声称找到同一个目标”。在这一步，方块折返、初始速度不准或持续未更新都会让预测逐渐偏离，最终触发拒绝；第 10 步会开始用匹配测量纠正预测。

## 两个可选小实验

### 1. 把 gate 调得很小

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args -p association_gate_m:=0.05
```

你会更常看到 `rejected_by_gate`。这说明门限过严，正常测量误差也被当成“不匹配”。

### 2. 把 gate 调得很大

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args -p association_gate_m:=2.00
```

你可能几乎总能看到 `matched`，但这不代表更好：很远的错误 cluster 也可能被错误接受。结束后不带参数重启即可恢复默认值。

## 本步故意还没有做什么

- 不更新位置、速度或协方差；这属于第 10 步的 Kalman measurement update。
- 不创建多个真实 `Track`，不分配 ID；这属于第 11 步。
- 不处理“一帧有 3 条轨迹、5 个 cluster”的全局一对一匹配；本步先验证单条预测的最近邻规则。
- 不实现匈牙利算法。最近邻 + gate 是首版可解释的 baseline，只有实验显示它不够用时才考虑更复杂方法。

## 本章必须懂的 ROS 2 / Nav2 知识

- 数据关联比较的所有位置必须在同一个 `frame_id`；本项目统一为 `odom`。
- 当前数组中的下标不能跨帧当 ID；ROS 消息数组每帧都可能重新排序。
- gate 是算法参数，应通过录包/回放验证，不要只凭一次 RViz 画面设定。

## 完成检查

- [ ] `colcon build --packages-select predictive_nav_tracking` 成功。
- [ ] 能看到 `CV predict` 和 `nearest-neighbor association` 两类日志。
- [ ] 理解 `matched` 与 `rejected_by_gate` 的区别。
- [ ] 知道最近的 cluster 不一定正确，因此需要 gate。
- [ ] 知道本步尚未更新状态，也尚未创建真实多目标 Track。
