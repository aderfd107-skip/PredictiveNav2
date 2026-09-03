# 01：观察 tracks 输入

## 这一步不写新代码

第三模块马上要预测“障碍物未来在哪里”。但预测并不能凭空开始：它必须先知道第二模块此刻到底给了什么信息。

本步只做一件事：观察正式输入接口：

```text
/dynamic_obstacles/tracks
```

确认它确实包含：稳定 ID、`odom` 坐标系、测量时间、位置、速度、协方差、尺寸和生命周期信息。

**输入接口**就是两个模块之间的约定。预测节点将来只能依赖这个 topic，不能再偷看：

- `/dynamic_obstacles/clusters`：它是单帧原始观测，已经由第二模块处理过；
- RViz Marker：它只是给人看的图形，不是机器接口；
- Gazebo 橙色真值 Marker：它只能用于人眼验证，严禁作为算法输入。

## 先复习：Track 与 prediction 的关系

```text
Track：      “现在在这里，以这个速度运动。”
Prediction： “如果短时间仍近似这样运动，0.2、0.4、0.6 秒后可能在这里。”
```

第二模块已完成：

```text
track_id = 17
当前位置 = (2.4, -1.1) m
当前速度 = (0.3, 0.0) m/s
测量时间 = 125.3 s
```

第三模块将以它为输入，在不读取任何 Gazebo 真值的前提下推出：

```text
t + 0.2 s → (2.46, -1.10) m
t + 0.4 s → (2.52, -1.10) m
...
```

这只是恒定速度（CV）的短时近似，不是承诺障碍物必然按直线走。

## 运行条件

要观察 tracks，第二模块的三个节点必须正在运行。每个终端先执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

终端 1：

```bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

终端 2：

```bash
ros2 run predictive_nav_perception scan_info_node
```

终端 3：

```bash
ros2 run predictive_nav_tracking tracking_node
```

不用启动预测节点，因为它还不存在。先等待终端 3 出现：

```text
published tracks | topic=/dynamic_obstacles/tracks | frame=odom | ...
```

这表示第二模块已经开始发布正式 Track。

## 第一个命令：先看“数据格式说明书”

新开终端 4，执行：

```bash
ros2 interface show predictive_nav_msgs/msg/TrackedObstacleArray
```

你会先看到：

```text
std_msgs/Header header
TrackedObstacle[] obstacles
```

含义是：一帧 Track 输出只有一个公共 `header`，其中有时间和坐标系；`obstacles` 列表里装这一时刻的多条 Track。

然后继续查看单条 Track：

```bash
ros2 interface show predictive_nav_msgs/msg/TrackedObstacle
```

它的核心字段是：

```text
uint32 track_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/Vector3 size
uint32 age
uint32 missed_frames
float32 confidence
```

先不要被 `PoseWithCovariance`、`TwistWithCovariance` 吓到。它们只是把“数值”和“这个数有多不确定”放在一起的 ROS 标准消息。

## 第二个命令：确认 topic 真实存在

```bash
ros2 topic info /dynamic_obstacles/tracks
```

正常时至少应有：

```text
Type: predictive_nav_msgs/msg/TrackedObstacleArray
Publisher count: 1
```

现在 prediction 节点还没创建，所以 `Subscription count` 可能是 0；这完全正常。等第 04 步创建订阅者后，它才会增加。

如果 `Publisher count: 0`，先回到 tracking 节点终端：确认它没有报错、确认感知节点正在发布 clusters，并确认你执行过 `source install/setup.bash`。

## 第三个命令：看一整帧真实数据

```bash
ros2 topic echo --once /dynamic_obstacles/tracks
```

输出很长是正常的，因为每条 Track 含有两个 6×6 协方差矩阵（各 36 个数）。你现在只需要按下面顺序看：

```text
header:
  stamp:       这批 Track 对应的 LiDAR 测量时间
  frame_id: odom

obstacles:
- track_id: 17
  pose:
    pose:
      position:
        x: ...
        y: ...
  twist:
    twist:
      linear:
        x: ...
        y: ...
  size:
    x: ...
    y: ...
  age: ...
  missed_frames: ...
  confidence: ...
```

### 读这一条消息时，每个字段是什么意思

| 字段 | 现在表示什么 | 第三模块将怎样使用 |
|---|---|---|
| `header.stamp` | 当前 Track 状态对应的测量时间，不是终端打印时间 | 预测未来时间轴的起点。 |
| `header.frame_id` | 当前应为 `odom` | 保证未来点与 Track 在同一连续坐标系。 |
| `track_id` | 跨帧稳定身份，例如 #17 | 未来预测也必须保留该 ID。 |
| `pose.pose.position.x/y` | Kalman update 后的二维当前位置 | 预测的起点。 |
| `twist.twist.linear.x/y` | 滤波后的二维速度 `vx/vy` | CV 模型中的速度项。 |
| `pose.covariance` | x/y 位置的不确定性 | 第 07 步把位置不确定性传播到未来。 |
| `twist.covariance` | vx/vy 速度的不确定性 | 后续预测不确定性的重要输入。 |
| `size.x/y` | 本次关联 cluster 的二维尺寸 | 未来风险计算需要障碍物尺寸，而不只看中心点。 |
| `age` | 这条 Track 经历了多少有效帧 | 调试新生/短命 Track。 |
| `missed_frames` | 连续多少帧没有匹配到可信观测 | 预测模块后续会据此做过期保护。 |
| `confidence` | 基于 miss 的观测新鲜度启发式值 | 不是物理身份正确率，也不是统计概率。 |

## 一个很重要的协方差复习

`PoseWithCovariance` 和 `TwistWithCovariance` 各有 36 个数，因为它们是 6×6 矩阵。当前 2D tracker 真正估计的是 x/y 与 vx/vy：

```text
pose.covariance 的 x/y 2×2 部分：下标 0、1、6、7
twist.covariance 的 vx/vy 2×2 部分：下标 0、1、6、7
```

其中：

```text
[ Pxx  Pxy ]
[ Pyx  Pyy ]
```

对角线是方差，非对角线是 x/y（或 vx/vy）的相关性。未观测的 z、姿态、角速度只在各自对角线使用大方差 `1e6`，其余交叉项为 0；不要把整张 6×6 矩阵都当成 `1e6`。

这一步你不需要手算协方差。只要知道：第 07 步不能只把中心点往前移，也必须说明“预测越远，位置有多不确定”。

## 第四个命令：确认频率，不把它写死

```bash
ros2 topic hz /dynamic_obstacles/tracks
```

一般会接近感知输入的约 10 Hz，但不要把它当作永远严格 10 Hz。tracking 节点只在：

```text
frame_id == odom
且 dt 是第一帧或有效 dt
```

时发布 tracks。暂停仿真、时间跳变或异常时间戳都可能使某一帧不发布，这是为了避免把旧 Track 假装成当前测量时间的状态。

## 你可能看到的几种正常现象

### `obstacles: []`

这不是 topic 坏了。它表示本帧没有存活 Track：可能刚启动、没有有效 cluster，或已有 Track 因连续 miss 被删除。等待几秒，或检查 cluster 输入。

### Track 数量很多，墙边也有 ID

当前 tracker 没有静态/动态分类。连续 cluster 都可能形成 Track，所以正式 tracks 也可能包含静态环境边缘。第 03 模块先对“现有 Track”预测；后续风险模块再明确哪些 Track 值得参与动态风险。

### `confidence: 1.0` 不代表 100% 正确

它目前是：

```text
confidence = 1 / (1 + missed_frames)
```

所以 `missed_frames == 0` 时为 1.0，只表示“本帧刚被观测到”，不表示关联绝不会错、更不表示它一定是真正的动态方块。

## 本步完成标准

- [ ] `ros2 interface show` 能看到 `TrackedObstacleArray` 与 `TrackedObstacle` 的字段；
- [ ] `ros2 topic info /dynamic_obstacles/tracks` 显示正确类型且 Publisher count 为 1；
- [ ] `ros2 topic echo --once` 中看到了 `header.frame_id: odom`、至少一条 Track 的 ID、位置、速度、尺寸、age、miss；
- [ ] 能说清 prediction 的起点是 `position + velocity + 测量时间`，不是 Gazebo 真值或 RViz Marker；
- [ ] 知道 `confidence` 是新鲜度启发式值，不是 100% 正确率。

完成后进入 [02_创建prediction包骨架](../02_创建prediction包骨架/README.md)。
