# 04：定义 Track 轨迹数据

## 这一小步完成什么

本步在 `tracking_node.cpp` 中新增了 `Track` 结构体，并新增：

```cpp
std::vector<Track> tracks_;
std::uint32_t next_track_id_{1U};
```

它们是 `TrackingNode` 的成员变量，意味着只要节点没有退出，数据就会一直留在内存中，能够跨越很多次 callback。

现在 `tracks_` 仍然是空的，因此日志中：

```text
active_tracks=0 | next_track_id=1
```

是**正确的预期结果**。本步只回答“一个 Track 应该长什么样”；后面才会回答“什么时候创建、怎样匹配、怎样更新、何时删除”。

## cluster 和 Track 到底有什么区别

| 项目 | `ObstacleCluster` | `Track` |
| --- | --- | --- |
| 存在多久 | 只属于当前一帧消息 | 从创建到删除，跨很多帧存在 |
| 来自哪里 | LiDAR 聚类结果 | 跟踪器内部维护的状态 |
| 有没有稳定 ID | 没有 | 有 `track_id` |
| 有没有速度 | 没有 | 状态中的 `vx`、`vy` |
| 是否有不确定性 | 没有 | `4 × 4` 协方差矩阵 |
| 会不会暂时看不见 | 本帧没有就是没有 | 可通过 `missed_frames` 暂时保留 |

可以把它理解为：

```text
cluster = 这一张照片中看到的一团点
Track   = 你在连续视频中持续盯住的同一个物体
```

## `Track` 中每个字段的意义

当前结构是：

```cpp
struct Track
{
  std::uint32_t track_id;
  Eigen::Vector4d state;
  Eigen::Matrix4d covariance;
  double size_x_m;
  double size_y_m;
  rclcpp::Time first_observation_stamp;
  rclcpp::Time last_observation_stamp;
  std::size_t age;
  std::size_t missed_frames;
};
```

### `track_id`

这是程序分配的稳定编号。第一条真正创建的轨迹会拿到 `1`，下一条拿 `2`，以此类推。

它和 `clusters[0]` 的数组下标完全不同：数组下标每帧会变，`track_id` 一旦分配就不应随意改变。

### `state`：位置与速度

`Eigen::Vector4d` 可以先理解为“恰好有 4 个 `double` 的数学向量”。四个位置固定含义为：

```text
state[0] = px：odom 中 x 位置，单位 m
state[1] = py：odom 中 y 位置，单位 m
state[2] = vx：x 方向速度，单位 m/s
state[3] = vy：y 方向速度，单位 m/s
```

代码用 `StateIndex` 给这些下标取了名字：

```cpp
Track::kPositionX
Track::kPositionY
Track::kVelocityX
Track::kVelocityY
```

以后读代码时，`state[Track::kVelocityX]` 比单独写 `state[2]` 更容易看懂，也更不容易写错。

本步所有 `state` 初值是 `0`。这不代表真实物体一开始就在原点、速度为 0；只是“还没从任何 cluster 初始化”的安全空状态。第 07～10 步才会赋予它真正的估计值。

### `covariance`：我们对状态有多不确定

`Eigen::Matrix4d` 是 `4 × 4` 的 `double` 矩阵。它会和 `[px, py, vx, vy]` 一一对应，表示：位置和速度各自的不确定性，以及它们之间的关系。

现在用单位矩阵作为可构建的占位初值。它**不是最终的噪声参数**；后面引入卡尔曼滤波时才会按位置和速度分别设置合理的初始不确定性。

你此刻只需记住一句话：

```text
state 说“我估计物体在哪、移动多快”
covariance 说“我对这个估计有多不确定”
```

### 尺寸、时间、`age`、`missed_frames`

- `size_x_m`、`size_y_m`：从 cluster 继承的当前尺寸估计；
- `first_observation_stamp`：这条轨迹第一次被看到的时间；
- `last_observation_stamp`：最近一次匹配到观测的时间；
- `age`：累计成功匹配观测的次数；
- `missed_frames`：连续多少帧没有匹配到这条轨迹。

这里的 `age` 是“看到并成功更新了多少次”，不是现实世界中物体活了多久。`missed_frames` 也不是错误：动态方块被墙挡住一两帧时，保留旧 Track 比马上删掉、重新分配 ID 更合理。

## `tracks_` 为什么必须是 Node 成员变量

对比下面两种位置：

```cpp
void cluster_callback(...) {
  std::vector<Track> tracks;  // 错误位置：回调结束后就被销毁
}
```

```cpp
class TrackingNode : public rclcpp::Node {
  std::vector<Track> tracks_; // 正确位置：节点活着，它就一直存在
};
```

局部变量 `tracks` 每来一帧就重新创建一个空列表，永远记不住上一帧的任何东西。成员变量 `tracks_` 才是“跨帧记忆”。这正是 C++ 类在本项目中的一个实际用途。

`next_track_id_` 同样是成员变量：即使旧轨迹被删除，也不会把编号重新从 1 开始，避免日志和后续预测模块混淆。

## 本次日志多出的字段

运行后，原先日志末尾会新增：

```text
active_tracks=0 | next_track_id=1
```

此时无论 cluster 有几个，`active_tracks` 都应保持 `0`。不要担心：第 11 步才会对未匹配 cluster 创建 Track；现在提前创建会让你跳过时间、预测和关联的学习过程。

## 构建与运行

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

随后保持第 01 模块的场景与 `scan_info_node` 运行，再启动：

```bash
ros2 run predictive_nav_tracking tracking_node
```

预期每十帧看到类似：

```text
cluster frame #10 | frame=odom | ... | clusters=... | ... |
active_tracks=0 | next_track_id=1
```

## 本步完成标准

- [ ] 构建成功。
- [ ] `tracking_node` 仍持续接收 `frame=odom` 的 cluster。
- [ ] 日志出现 `active_tracks=0 | next_track_id=1`。
- [ ] 我能解释 cluster 是单帧输入，Track 是跨帧持久状态。
- [ ] 我知道 `state=[px, py, vx, vy]`，且 covariance 代表不确定性。

## 本章关联的 ROS 2 / C++ 知识

**本章必须懂**：`struct` 用来把一条 Track 的相关数据放在一起；`std::vector<Track>` 是会变长的轨迹列表。把它作为 Node 成员变量，才能跨 callback 保存状态。Eigen 是后续卡尔曼滤波用的矩阵库。

**可选扩展**：现在不用推导协方差矩阵，也不用自己学完整 Eigen API。下一步先处理真实消息的时间戳和 `dt`，再让这些字段逐个开始拥有实际意义。
