# 第二模块：从障碍物候选观测到多目标轨迹

## 这一模块接在第 01 模块哪里

第 01 模块已经持续发布：

```text
/dynamic_obstacles/clusters
  header.stamp / header.frame_id=odom
  clusters[]：centroid、size_x_m、size_y_m、point_count
```

但每一帧的 cluster 都没有名字。例如，上一帧第 2 个 cluster 与下一帧第 2 个 cluster，不一定是同一个物体；数组下标会随视野、遮挡和聚类结果改变。

本模块要新增 `predictive_nav_tracking` C++ 包，最终形成：

```text
/dynamic_obstacles/clusters
  ↓
predictive_nav_tracking
  ↓
/dynamic_obstacles/tracks
  每条轨迹：稳定 ID、位置、速度、协方差、尺寸、命中次数、丢失次数
```

## 最终要解决的真实问题

连续两帧中，机器人看到三个 cluster：

```text
时刻 t：      A(1.0, 2.0)    B(3.0, 1.0)    C(5.0, 4.0)
时刻 t + dt： A(1.1, 2.0)    B(3.0, 1.0)    C(4.9, 4.0)
```

跟踪器要做的不是把数组第 0、1、2 项机械地对应起来，而是回答：

```text
哪个新观测最可能属于已有 track #17？
哪个观测是新出现的物体？
哪个旧轨迹这次暂时没看到？
```

每条轨迹使用恒定速度（CV，Constant Velocity）状态：

```text
x = [位置 x, 位置 y, 速度 vx, 速度 vy]
```

卡尔曼滤波会将上一帧状态的预测与当前激光观测结合起来，避免直接用两帧坐标相减而产生非常抖动的速度。

## 小步骤顺序

1. [00_这一模块要做什么](00_这一模块要做什么/README.md)：先分清 cluster、track、prediction 的职责。
2. [01_观察cluster消息](01_观察cluster消息/README.md)：不写代码，确认第 01 模块的正式输入接口。
3. [02_创建tracking包骨架](02_创建tracking包骨架/README.md)：建立独立的 C++ 跟踪包。
4. [03_订阅cluster并打印](03_订阅cluster并打印/README.md)：确认跟踪节点收到的是 `odom` 中、带时间戳的观测。
5. [04_定义Track轨迹数据](04_定义Track轨迹数据/README.md)：明确一条轨迹需要保存哪些长期状态。
6. [05_处理时间戳与dt](05_处理时间戳与dt/README.md)：不假定固定 10 Hz，使用真实测量时间计算 `dt`。
7. [06_单目标连续观测与朴素速度](06_单目标连续观测与朴素速度/README.md)：先直观看懂“速度为何会抖”，为卡尔曼滤波做准备。
8. [07_引入CV卡尔曼状态](07_引入CV卡尔曼状态/README.md)：使用 Eigen 表示位置、速度和协方差。
9. [08_预测已有轨迹](08_预测已有轨迹/README.md)：让每条轨迹按真实 `dt` 走到当前时刻之前的预计位置。
10. [09_最近邻数据关联](09_最近邻数据关联/README.md)：用距离门限把 cluster 与预测后的轨迹匹配。
11. [10_卡尔曼更新与速度估计](10_卡尔曼更新与速度估计/README.md)：用匹配到的观测修正位置、速度与不确定性。
12. [11_新生轨迹与丢失管理](11_新生轨迹与丢失管理/README.md)：处理新物体、短暂遮挡和应删除的旧轨迹。
13. [12_发布TrackedObstacle消息](12_发布TrackedObstacle消息/README.md)：定义并发布正式 `/dynamic_obstacles/tracks` 接口。
14. [13_RViz验证与动态方块观察](13_RViz验证与动态方块观察/README.md)：在 RViz 显示 ID、速度箭头和轨迹状态。
15. [14_rosbag回放与排错](14_rosbag回放与排错/README.md)：录制、回放固定输入，复现 ID 切换和参数问题。

## 本模块刻意不做什么

- 不直接用 Gazebo actor 的位置、速度或橙色真值 Marker；
- 不把“cluster 数组下标相同”误当作同一个物体；
- 不一开始实现匈牙利算法、JPDA、深度学习跟踪或复杂 DBSCAN；
- 不在本模块预测未来很久，也不修改 Nav2 控制器。

第一版采用可解释、可调试的**预测后最近邻贪心匹配 + CV 卡尔曼滤波**。当它稳定运行后，才值得用 benchmark 比较匈牙利算法等替代方案。

## 本模块完成时，你应该能说清

1. cluster 是单帧几何观测，track 是跨帧维护的状态；
2. 为什么必须使用消息时间戳计算 `dt`，而不是写死 `0.1 s`；
3. 为什么先预测，再关联，再更新；
4. 为什么一个暂时没被观测到的 track 不能立刻删除；
5. track 的速度与协方差如何成为第 03 模块轨迹预测的输入。

现在先从第 00 步开始。每一步的代码都会在实际开始该步时生成；你不需要提前手写任何卡尔曼滤波或 C++。

---

## 学完第二模块后：你实际完成了什么

现在这 14 步已经完成。不要把它简单概括成“我写了一个卡尔曼滤波器”；你实际建立的是下面这条可运行的数据链：

```text
/scan
  ↓  C++ 感知：有效量程、TF 到 odom、顺序欧氏聚类
/dynamic_obstacles/clusters
  ↓  C++ 跟踪：真实 dt → CV predict → gate 内一对一关联 → Kalman update
tracks_（节点内部的跨帧记忆）
  ↓
/dynamic_obstacles/tracks          正式机器接口
  ↓
/dynamic_obstacles/track_markers   RViz 调试图形
```

你还完成了：动态 actor 场景观察、青色 cluster 与 Track 的对照、rosbag 录制与只回放 cluster 的可复现排错流程。

## 最重要的系统图

```text
单帧 cluster                         跨帧 Track
中心、尺寸、点数、时间戳     →       ID、[px, py, vx, vy]、P、age、miss

第 k 帧观测
      ↓
1. 读取 header.stamp，计算真实 dt
      ↓
2. 对所有旧 Track 做 CV 预测
      ↓
3. 在 gate 内建立 Track—cluster 候选对
      ↓
4. 按距离贪心接受一对一匹配
      ↓
5. 已匹配：Kalman update；未匹配旧 Track：miss + 1；未匹配 cluster：birth
      ↓
6. 超过 max_missed_frames：删除 Track
      ↓
7. 发布正式 tracks 与仅供 RViz 使用的 Marker
```

记住这个顺序：**预测 → 关联 → 更新 → 生命周期**。如果先拿新观测更新旧 Track、再猜它属于谁，逻辑就会混乱。

## 你已经接触到的 C++，不需要死背但要能看懂

| C++ 写法 | 在项目中的用途 | 你应该理解到什么程度 |
|---|---|---|
| `struct Track` | 把一个目标长期需要的数据放在一起 | 知道成员变量会随 `tracks_` 一直保存。 |
| `enum StateIndex` | 给状态向量的下标命名 | `state[2]` 不直观，`kVelocityX` 更不容易写错。 |
| `std::vector<Track>` | 保存数量会变化的多个 Track | birth 用 `push_back`，删除用 `erase`。 |
| `const T &` | 不复制大消息、且承诺不修改输入 | 明白它不是“额外造一个新对象”。 |
| `std::size_t` / `uint32_t` | 表示数组下标、计数和 ROS ID | 知道负数不能直接转无符号数。 |
| `std::sort` + lambda | 对候选 pair 按距离排序 | 排的是完整 Candidate，不是把 track/cluster 下标弄丢。 |
| `Eigen::Vector4d` / `Matrix4d` | 表示 CV 状态和协方差 | 不要求手推全部矩阵，但要知道状态与不确定性是一组。 |
| `rclcpp::Publisher` / subscription | 节点间通过 topic 通信 | 内存里的 `tracks_` 不能被其他节点直接访问，必须发布消息。 |

你现在不需要脱离项目手写完整 Tracker；但应该能沿着 `tracking_node.cpp` 从 callback 找到上述每一步在何处发生。

## 这章必须能讲清的 ROS 2 / 机器人概念

### 1. 为什么状态放在 `odom`，不是 `lidar_link` 或 `map`

- `lidar_link` 会随小车移动；直接在其中跨帧求速度会把自车运动混进去；
- `map` 会被 AMCL 通过 `map → odom` 修正，短时间可能跳变；
- `odom` 对局部运动连续，适合这版短时 Track、速度和预测。

### 2. 为什么 `dt` 来自 header，而不是“默认 10 Hz”

消息到达频率会受仿真、调度、暂停和 rosbag 回放影响。`header.stamp` 代表这束 LiDAR 实际采样的时间；CV 预测和速度都必须依据它。慢放 rosbag 不应把真实速度错误地变成一半。

### 3. 为什么 QoS 和 frame 会造成“topic 存在但效果不对”

- 高速 `/scan`、clusters、tracks 使用面向最新数据的 `SensorDataQoS`；
- 静态 `/map` 要使用 `Transient Local` 才能让晚启动的 RViz 收到保留地图；
- Marker 显示需要正确 topic、namespace、frame 和时间，RViz 有画面不等于算法已正确。

### 4. 为什么 Track 和 Marker 必须分开

`/dynamic_obstacles/tracks` 是给下游程序读的结构化消息；`/dynamic_obstacles/track_markers` 只为了人眼调试。下一模块只能订阅 tracks，不能把 Marker 当作预测输入。

## 这章中你亲自遇到、以后很有价值的工程问题

| 现象 | 最终判断 | 可复述的经验 |
|---|---|---|
| RViz `Map` 显示 `No map received`，但 Topic 是 OK | `/map` 的 QoS durability 不匹配 | DDS 发现 topic 不等于收到了历史静态消息。 |
| 激光整体偏离地图、小车中途停住 | AMCL 的 `map → odom` 定位错位，随后 Nav2 action 被中止 | TF 链存在不等于定位正确，要看 scan 是否贴图。 |
| `/cmd_vel_safe` 全是 0 | 先确认 action status，而不是先怪安全层 | `controller_server active` 与某次 action `ABORTED` 可以同时成立。 |
| 青色 cluster 突然不显示 | RViz MarkerArray topic 被保存成默认值 | 先查 Publisher count，再查 RViz 的 Topic/Enabled/namespace。 |
| 全量 Track 文本淹没 RViz | 墙体/家具也被跟踪，尚无动态性分类 | 用速度阈值做**显示筛选**，但不能把它说成动态识别算法。 |
| 36 项协方差不知如何填 | 未观测变量只有对角线给大方差 | 大的 off-diagonal 会虚构相关性，未知交叉项应为 0。 |

这些不是“无关紧要的小 bug”。它们正是你未来面试能说明自己实际运行、排错和做工程取舍的证据。

## 当前实现的诚实边界

已实现：

- CV 卡尔曼预测/更新；
- 欧氏距离 gate 内的贪心一对一关联；
- birth、miss、超阈值删除；
- Track ROS 消息、RViz 调试与 rosbag 回放流程。

尚未实现，因此不能夸大：

- cluster 是静态还是动态的可靠分类；
- 目标交叉、贴近、遮挡时不发生 ID switch；
- Mahalanobis gate、Hungarian 全局分配、tentative/confirmed Track；
- 未来轨迹预测、动态风险打分、对 Nav2 控制决策的影响；
- 真实硬件上的 `Q/R/gate/max_missed_frames` 标定。

其中第一版的贪心关联是一个可解释 baseline，不是最终答案。先录下会出错的 rosbag，再在同一输入上比较升级方法，才是正确的深化顺序。

## 进入第三模块前的完成检查

在继续 [第三模块：轨迹预测](../03_轨迹预测/README.md) 前，建议你确认下列事项：

- [ ] 能用自己的话解释 cluster 与 Track 的区别；
- [ ] 能在 `tracking_node.cpp` 中指出 predict、association、Kalman update、birth/miss/expiry 的位置；
- [ ] `/dynamic_obstacles/tracks` 能稳定发布，header frame 为 `odom`；
- [ ] RViz 能看见青色 cluster 和简洁的运动候选 Track Marker；
- [ ] 已至少录一份包含 `/dynamic_obstacles/clusters` 的 rosbag，并会“只回放 clusters + 新 tracker”；
- [ ] 知道当前没有动态性分类，也不把 Gazebo 真值 Marker 当算法输入；
- [ ] 能解释一次你遇到的 QoS、TF 或 action 状态排错。

## 60 秒项目表达（当前阶段）

> 我先把 2D LiDAR scan 转到连续的 odom 坐标系并聚类，得到每帧障碍物候选观测。跟踪模块以 `[px, py, vx, vy]` 为 CV 卡尔曼状态，严格用消息时间戳计算 dt；每帧先预测，再在距离 gate 内做一对一贪心关联，匹配后 Kalman update，未匹配的 Track 用 missed 计数管理，新观测创建递增 ID。结果通过 `/dynamic_obstacles/tracks` 发布，并用 RViz 和 rosbag 在固定输入上验证。当前这是可解释 baseline，交叉遮挡下的 ID switch、动态性分类和未来轨迹预测将作为下一阶段改进，而不是宣称已经全部解决。

下一步进入第三模块：把每条当前 Track 的状态和协方差推到多个未来时刻，得到能够被风险评价使用的**带时间的预测轨迹**。
