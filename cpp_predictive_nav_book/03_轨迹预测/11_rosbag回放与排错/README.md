# 11：rosbag 回放与排错

## 一句话理解 rosbag

`rosbag` 不是屏幕录像。它把 ROS 2 的话题消息连同原始时间戳保存到磁盘，之后按原先顺序重新发布。

对轨迹预测来说，它的价值是固定住输入：同一份 `/dynamic_obstacles/tracks` 可以反复喂给 prediction_node。这样你改了预测参数后，比较的是算法差异，而不是“这次 Gazebo 方块碰巧走了另一条路”。

```text
一次运行里看到预测异常
        ↓
录下 tracks、TF、时钟等证据
        ↓
停止仿真，只回放同一份 tracks
        ↓
prediction_node 重新生成 predictions / markers
        ↓
公平比较参数、定位时间戳或 frame 问题
```

因此第 11 步**不新增 C++ 算法**；它给第 01–10 步建立可复现实验和排错流程。这也是第二章 rosbag 步骤的延续：第二章固定 cluster 来测 tracker，本章固定 tracks 来测 predictor。

## 两类包，两个用途

| 类型 | 录什么 | 用来回答什么 |
|---|---|---|
| 证据包 | `/clock`、scan、TF、cluster、tracks、预测、速度命令等 | 当时整套导航系统发生了什么？ |
| 最小预测回放包 | 最关键是 `/clock` 与 `/dynamic_obstacles/tracks` | 相同 Track 输入下，预测节点输出是否一致、参数变化有什么影响？ |

记录预测输出和 Marker 是为了事后对照；但**重新运行 prediction_node 时不能同时回放旧 predictions/markers**，否则新旧两个输出会混在一起，观察结果没有意义。

## 录制一份预测证据包

先启动第 10 步的四个终端，确认动态方块、Track 和青色预测轨迹正常出现。新开终端 5：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p bags
```

然后录 20–30 秒。可以先让动态方块自行运动；若想同时保留导航现象，再在 RViz 发一个目标点。

```bash
ros2 bag record -o bags/step11_prediction_case_01 \
  /clock \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /dynamic_obstacles/clusters \
  /dynamic_obstacles/tracks \
  /dynamic_obstacles/predictions \
  /dynamic_obstacles/prediction_markers \
  /cmd_vel \
  /cmd_vel_safe \
  /navigate_to_pose/_action/status
```

结束时按一次 `Ctrl+C`，并等待 rosbag 显示写盘结束。不要直接关闭窗口；这样可能来不及写 `metadata.yaml`。

检查包是否完整：

```bash
ros2 bag info bags/step11_prediction_case_01
```

至少应看到 `/clock`、`/dynamic_obstacles/tracks` 和 `/dynamic_obstacles/predictions`，且 duration、message count 都不是 0。

## 最小回放：只重新验证预测节点

目标是让 bag 成为唯一的 tracks 输入，再让**新启动**的 prediction_node 产生新的预测结果。

### 1. 停掉实时输入

停止 Gazebo、scan_info_node、tracking_node，以及旧 prediction_node。否则实时 tracker 与 rosbag 会同时往 `/dynamic_obstacles/tracks` 发消息，输入混在一起。

最小回放不需要 Gazebo、地图、AMCL 或 Nav2。若只想看轨迹，可以单独开 RViz，并把 Fixed Frame 设置为 `odom`。

### 2. 先启动 prediction_node

终端 A：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node --ros-args -p use_sim_time:=true
```

必须先启动它：它使用 `/clock` 和 tracks 的 `header.stamp` 判断输入新鲜度。如果后启动，包开头的一段消息已经过去，可能看不到完整结果。

### 3. 只播放时钟和 tracks

终端 B：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 bag play bags/step11_prediction_case_01 \
  --topics /clock /dynamic_obstacles/tracks
```

不要播放 `/dynamic_obstacles/predictions` 或 `/dynamic_obstacles/prediction_markers`。两者应由刚启动的 prediction_node 重新生成。

若只想慢放观察，可以加：

```bash
ros2 bag play bags/step11_prediction_case_01 \
  --topics /clock /dynamic_obstacles/tracks \
  --rate 0.5
```

`--rate 0.5` 只让消息到达得更慢；消息自身的 `header.stamp` 不会被改写。预测的时间基准仍来自 ROS 时间和 Track 时间戳，而不是人的播放速度。

## 回放时看什么

prediction 终端应周期性出现：

```text
track frame | stamp=... | frame=odom | track_count=...
published predictions | trajectories=... | points_per_trajectory=10
CV preview | id=... | now=(...) | vel=(...) | t=2.00 s -> (...)
```

重点按以下顺序定位问题：

| 现象 | 先检查 | 常见原因 |
|---|---|---|
| 没有任何输出 | `ros2 bag info` 是否真的有 tracks；节点是否先于播放启动 | bag 少录了 topic，或播放过早开始 |
| `track frame rejected` | frame、`header.stamp`、`/clock` | 不是 `odom`、时间为 0、时间未推进、输入太旧 |
| `trajectories=0` | `track quality gate` 日志 | 年龄、置信度、miss 次数、速度或协方差未通过门限 |
| RViz 无图形但 prediction 有输出 | Marker topic、Fixed Frame、namespace | 通常是显示配置，不是 CV 算法 |
| 同一 bag 改参数后画面不同 | 确认只回放 tracks，且只运行一份 prediction_node | 输入混入实时 tracker 或旧输出 |

在独立 RViz 中添加：

```text
MarkerArray → /dynamic_obstacles/prediction_markers
```

并把 Fixed Frame 设为 `odom`。没有地图、机器人模型和 Gazebo 真值框是正常的：最小回放的目的只是验证 predictor，不是重建整个仿真。

## 用同一份 bag 比较一个参数

例如先跑默认基线，再仅改变未知加速度标准差：

```bash
ros2 run predictive_nav_prediction prediction_node --ros-args \
  -p use_sim_time:=true \
  -p prediction_acceleration_stddev_mps2:=0.5
```

两次都回放同一个 `step11_prediction_case_01`，比较：

- 预测中心线：在 CV 模型中应保持相同，因为状态和速度输入没有变；
- 紫色终点不确定性范围：较小的加速度标准差通常让远期范围变小；
- 通过质量门限的轨迹数量：若只改该参数，数量通常不应无故变化；
- 是否出现 NaN、时间倒退、frame reject 或旧 Marker 残留。

一次只改一个参数，并在实验记录中写明 bag 名称、参数和结论。不要只凭某一次画面“看起来更好”就说模型更准确。

## 关于时间冻结的正常现象

bag 播放完时 `/clock` 会停在包的最后时刻，所以 simulation time 不再前进。这时预测节点的 stale watchdog 也不会继续变老；这是回放结束后的正常行为，不代表 watchdog 失效。

要验证“实时输入突然中断”的 stale 检测，应在 Gazebo 仍运行时停止 tracking_node，让 `/clock` 继续走，再观察 prediction_node 的 stale warning。这个测试与离线 bag 回放是两个不同场景。

## 为什么不把 bags 上传到 GitHub

scan、TF、Marker 等原始话题很容易让单个 bag 变成几百 MB。仓库的 `.gitignore` 已排除 `bags/`：

- GitHub 保存源码、配置、README、命令和实验结论；
- 原始 bag 保存到本地或网盘，并用名称/日期关联实验记录；
- 面试或答辩展示 `ros2 bag info`、回放截图和可复现步骤即可，不需要把大文件塞进仓库。

## 本步完成边界

完成第 11 步后，你能把一次动态障碍物预测运行录成证据包；停止仿真后只回放 `/clock + /tracks`，让 prediction_node 重建其输出，并据此复现和定位问题。

这还不是预测精度 benchmark。后续实验模块会再统计位置误差、速度误差、覆盖率和计算时延；现在先保证任何异常都有一份可重复的输入证据。
