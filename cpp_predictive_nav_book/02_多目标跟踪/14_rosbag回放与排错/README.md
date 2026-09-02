# 14：rosbag 回放与排错

## 先用一句话理解 rosbag

`rosbag` 不是屏幕录像。它把 ROS 2 话题消息及其原始时间戳录到磁盘；之后可以按照原来的先后顺序重新发布这些消息。

对本项目来说，它解决的问题是：

```text
动态方块这次经过 → Track ID 突然切换 / 速度异常 / Track 消失
                    ↓
不录包：只能等待方块下一次经过，输入已经变了
录包后：同一批 cluster 可重复播放，参数改变前后可以公平比较
```

因此第 14 步不是“再写一个算法”，而是为已有的跟踪算法建立可复现实验输入。它是后面优化关联、预测和风险评价时的重要基础。

## 这一步的两种 rosbag 用法

不要把所有 bag 都当成同一种用途。

| 名称 | 录什么 | 用来回答什么问题 |
|---|---|---|
| **证据包** | `/scan`、cluster、Track、TF、速度、动作状态等 | 当时整个系统发生了什么？Nav2 为什么停止？ |
| **最小回放包** | `/dynamic_obstacles/clusters` | 在完全相同的感知输入下，tracker 改参数后会怎样？ |

本步先做两者兼顾的证据包；回放 tracker 时只播放其中的 cluster。原因是 tracker 会自己重新发布 `/dynamic_obstacles/tracks`，不能同时再把录下来的旧 tracks 播放出来。

## 先理解要录的接口

```text
/scan
  ↓  scan_info_node
/dynamic_obstacles/clusters       ← tracker 的正式输入，最关键
  ↓  tracking_node
/dynamic_obstacles/tracks         ← 当时的 baseline 输出，用于事后对照
  ↓
/dynamic_obstacles/track_markers  ← 仅供 RViz 观看
```

另外录 `/odom`、`/tf`、`/tf_static`，是为了未来能解释“这个 cluster/Track 当时处在哪个坐标系”；录 `/cmd_vel`、`/cmd_vel_safe` 和 action status，是为了关联导航停止现象。

## 录制前的场景要求

先运行第 13 步的三个终端：

```text
终端 1：Gazebo + AMCL + Nav2 + RViz（enable_dynamic_obstacle:=true）
终端 2：scan_info_node
终端 3：tracking_node
```

在 RViz 中先确认：

- 青色 cluster 能出现；
- 运动候选 Track 框、ID、速度箭头能出现；
- 动态 actor 正在移动；
- 没有故意让 `2D Pose Estimate` 覆盖自动 AMCL 初始位姿。

只有输入链路正常时录的 bag 才能作为可比较的基线。若当前就是为了保留异常，也可以录，但要在文件名和实验笔记中明确写出异常类型。

## 第一次录制：保存完整证据

新开**终端 4**，先执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p bags
```

然后开始录制。以下名称中的 `step14_dynamic_case_01` 是你第一次可复现实验的名字；同名目录已存在时 rosbag 会拒绝覆盖，因此下一次改为 `case_02`、`case_03`。

```bash
ros2 bag record -o bags/step14_dynamic_case_01 \
  /clock \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /dynamic_obstacles/clusters \
  /dynamic_obstacles/tracks \
  /dynamic_obstacles/cluster_markers \
  /dynamic_obstacles/track_markers \
  /cmd_vel \
  /cmd_vel_safe \
  /navigate_to_pose/_action/status
```

录制开始后，不需要急着点击目标。先录 15–30 秒动态方块的正常运动；再可选地发一个 Nav2 Goal，让 bag 中有一次完整导航过程。

结束时在终端 4 按一次 `Ctrl+C`，然后等待它显示写盘完成再关闭终端。不要直接关窗口，否则 metadata 可能没有写完整。

## 确认 bag 真的可用

录制结束后执行：

```bash
ros2 bag info bags/step14_dynamic_case_01
```

你至少应看到：

```text
/dynamic_obstacles/clusters
/dynamic_obstacles/tracks
/odom
/scan
```

并检查 duration 不是 `0`、message count 不是 `0`。若 `clusters` 为 0，说明当时 `scan_info_node` 没有运行，录到的包不能用于 tracker 回放。

## 最小回放：固定 cluster 输入，重新运行 tracker

这是本步最重要的操作。目的是让**新启动的** tracker 只收到录包里的 `/dynamic_obstacles/clusters`，再生成一遍 tracks。

### 1. 先停掉实时 tracker

在终端 3 按 `Ctrl+C`。否则实时 `scan_info_node` 和 bag 会同时向 `/dynamic_obstacles/clusters` 发布消息，输入混在一起，回放实验失去意义。

实时 Gazebo、Nav2、感知节点也建议全部停止；本次只验证 tracker 时不需要它们。

### 2. 终端 A：启动新的 tracker

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_tracking tracking_node
```

### 3. 终端 B：只播放 cluster

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play bags/step14_dynamic_case_01 \
  --topics /dynamic_obstacles/clusters
```

**不要**在这个命令中播放 `/dynamic_obstacles/tracks` 或 `/dynamic_obstacles/track_markers`：它们应该由新 tracker 重建。否则 RViz/`ros2 topic echo` 会混入“旧输出”和“新输出”，无法判断参数改变是否有效。

## 回放时怎样观察

先看 tracker 终端中的：

```text
cluster frame ... dt=0.100 s (valid)
track lifecycle ...
track[0] | id=... | position=... | velocity=...
```

重点记录：

| 现象 | 先看什么 | 可能的后续方向 |
|---|---|---|
| ID 在目标交叉/靠近时改变 | 关联距离、同帧 cluster 数 | 第 15 步再比较 Mahalanobis/Hungarian。 |
| 静止墙体出现很大速度 | cluster 抖动、错误关联、`R/Q` | 先保留 bag，再比较滤波参数。 |
| Track 一闪而过 | `missed_frames`、`max_missed_frames`、cluster 分裂 | 用相同 bag 调阈值并对比。 |
| `dt` 不为原来的约 0.1 秒 | bag 的 header 是否异常 | 先查源数据，不能把问题归咎于播放速度。 |

可以用较慢播放速度观察，但 tracker 仍应使用消息 header 中的原始测量时间：

```bash
ros2 bag play bags/step14_dynamic_case_01 \
  --topics /dynamic_obstacles/clusters \
  --rate 0.5
```

`--rate 0.5` 只让消息“到达得更慢”，不应改变 `header.stamp`。本项目的 `dt` 来自 cluster header，所以正确实现不该因肉眼慢放而把速度算成一半；这正是第 05 步不用 wall-clock 的实际价值。

## 怎样比较一次参数修改

一次只改**一个**参数，并给运行结果取新名字。例如先记录 baseline：

```text
case_01_baseline
association_gate_m = 0.40
measurement_position_stddev_m = 0.15
max_missed_frames = 5
```

再仅改变 gate：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args \
  -p association_gate_m:=0.30
```

然后播放**同一份** bag。比较至少包括：

- 同一运动方块的 ID 是否更稳定；
- 是否出现更多 `missed` 或 Track 断裂；
- 墙体/静态杂物是否更容易抢走关联；
- 速度箭头是否更合理。

不要只因为一次 RViz 画面“看起来顺眼”就宣称参数更好。应把 bag 名称、参数和观察结论写到实验记录中；以后才能做 benchmark 表格。

## RViz 回放是可选项

最小 tracker 回放不需要 Gazebo、地图或 AMCL。若要可视化，可单独启动 RViz，将 Fixed Frame 设为 `odom`，再添加：

```text
MarkerArray → /dynamic_obstacles/track_markers
```

此时看到的是新 tracker 从 bag cluster 重建的 Marker。没有地图、机器人模型或 Gazebo 橙色真值框是正常的；这个模式的目的只是重复验证跟踪器。

## 为什么 `bags/` 不上传 GitHub

原始 scan、TF、Marker 等 rosbag 往往很大，且每次实验都可能生成数百 MB 或更多。仓库已把 `bags/` 加入 `.gitignore`：

- GitHub 保存源码、README、参数、实验表格和少量文字结论；
- 原始 rosbag 本地保存、备份到网盘/专用存储，或只在需要时提供下载链接；
- 简历/答辩中可以展示 `ros2 bag info`、运行录像、参数表与复现步骤，不需要把大文件硬塞进代码仓库。

## 常见错误

### `ros2 bag play` 后 tracker 没有任何输出

先确认：

```bash
ros2 bag info bags/step14_dynamic_case_01
```

是否真的包含 `/dynamic_obstacles/clusters`。然后确认播放命令没有拼错 topic，并且 tracker 在 `ros2 bag play` 之前已经启动。

### 回放时有两个 cluster Publisher

这是实时 `scan_info_node` 和 bag 同时在发布。停止实时感知节点，只保留 bag 播放者；最小回放的输入必须唯一。

### 回放时 `/tracks` 数量看起来翻倍

通常是同时播放了录下来的 `/dynamic_obstacles/tracks`、又启动了新 tracker。只播放 clusters，让新的 tracker 独占 tracks 输出。

### bag 目录无法重新录制

rosbag 默认不覆盖已有输出目录。保留旧包并换一个新名称，例如 `step14_dynamic_case_02`；不要为了录新包随手删除旧的、还没有分析的证据。

## 本章必须懂的 ROS2 / Nav2 知识

- rosbag 记录的是带时间戳的 ROS 消息，不是视频；
- 回放实验必须确保输入 topic 只有一个发布者；
- 输入、旧输出、新输出要区分，否则会把两次算法结果混在一起；
- `header.stamp` 与播放 wall-clock 是两回事，跟踪 `dt` 应来自测量时间；
- 大型 bag 不适合直接提交 Git，源码仓库应保存复现方法与实验元数据。

## 本步完成的边界

完成本步后，你能录一段固定 cluster 输入、在不启动 Gazebo 的条件下重复运行 tracker，并公平比较一个参数修改前后的输出。

这还不是完整 benchmark：尚未自动统计 ID switch、位置误差、速度误差或成功率；这些将在第 05 模块的实验验证中系统整理。
