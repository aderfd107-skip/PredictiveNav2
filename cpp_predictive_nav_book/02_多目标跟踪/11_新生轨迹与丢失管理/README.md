# 11：新生轨迹与丢失管理

## 先用一句人话说明

到第 10 步为止，我们只是在一个教学用的 `debug_cv_state_` 上，把“预测 → 找最近 cluster → Kalman 更新”看明白了。它的 ID 固定为 0，不能代表真实世界里有很多障碍物。

这一步开始维护真正的 `tracks_` 数组。每一个 `Track` 都有自己的递增 ID，例如 `1`、`2`、`3`。核心规则只有两条：

- 当前帧某个 cluster 没有匹配到旧 Track：它**可能是新物体**，创建一个新 Track；
- 某个旧 Track 本帧没有匹配到 cluster：它**可能只是暂时遮挡或漏检**，先保留，`missed_frames` 加一；连续丢失过多帧才删除。

```text
当前 clusters                         已有 tracks
    ○  ○  ○                              #1  #2
       │  │                                │   │
       │  └─ 未匹配 ───────────────→  新建 #3
       └──── 匹配 ──────────────────→  更新 #1
                                          #2 没匹配到
                                          ↓
                                    missed_frames += 1
                                    超过阈值才删除
```

这就是“多目标”的第一层含义：程序不再依赖 cluster 数组下标，而是维护跨帧存在的 Track 和稳定 ID。

## 先认识四个名词

| 名词 | 白话含义 |
|---|---|
| cluster | LiDAR 在**这一帧**看到的一团点；它没有稳定身份。 |
| Track | 程序跨帧保存的“某个可能物体”的状态：ID、位置、速度、协方差、尺寸和历史信息。 |
| birth（新生） | 没有匹配到旧 Track 的 cluster 创建新 Track。 |
| missed frame（丢失帧） | 某个 Track 本帧没有可信匹配，但还不急着删除。 |

请特别区分：**Track 是跟踪器的假设，不是传感器直接给出的真相。** 一个刚出生 Track 仍可能是聚类抖动、静态物体，或以后会消失的误检；当前版本先把生命周期做正确，后面再讨论确认机制与“它是否真的在动”。

## 本步真实的每帧流程

只有 cluster 消息的坐标系是 `odom`、且时间间隔有效时，真实 Track 才会更新。顺序如下：

1. 对所有现有 Track 使用 CV 模型预测到当前消息时间；
2. 枚举“Track 预测位置—当前 cluster 中心”的距离；超过 `association_gate_m` 的组合直接排除；
3. 从剩余组合中按距离从小到大选择，并保证**一个 Track 最多匹配一个 cluster，一个 cluster 最多更新一个 Track**；
4. 匹配成功的 Track 做 Kalman update，清零 `missed_frames`；
5. 未匹配 Track 的 `missed_frames` 加一；
6. `missed_frames > max_missed_frames` 时删除该 Track；
7. 对未匹配的有效 cluster 创建新 Track，速度初始化为 `(0, 0)`，速度协方差保持较大。

第 7 步为什么不把第 06 步的两帧差分速度塞进去？因为新 cluster 还没有经过可靠关联，差分可能来自两个不同物体，得到的是“假速度”。正式 Track 从速度为 0、但“对速度很不确定”开始，随后依靠连续的 Kalman update 逐步估计速度。

## 本步的关联方法：基础但诚实

代码采用“**距离 gate 内的贪心一对一匹配**”：所有候选 pair 先按距离排序，最短 pair 优先接受；接受后对应的 Track 和 cluster 都不能再被其他 pair 使用。

它比“每个 Track 自己找一个最近 cluster”多了一条重要保护：同一个 cluster 不会同时更新两个 Track。

但它仍然不是 Hungarian 算法，也没有 Mahalanobis distance。两个目标靠近、交叉或被遮挡时，仍可能发生 `identity switch`（ID 对应到另一个物体）。这是当前版本真实存在的边界，不要在面试中说“最近邻能保证同一物体”。后续用 rosbag 评估后，再比较更复杂的关联方法。

## 本步写入了哪些代码

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)

| 函数 / 数据 | 作用 |
|---|---|
| `make_new_track()` | 由一个未匹配 cluster 创建新 Track，分配递增 ID，初速度为 0。 |
| `predict_all_tracks()` | 对所有已有 Track 做 CV 预测；若数值出现 NaN/Inf，丢弃该异常 Track。 |
| `associate_tracks_one_to_one()` | 构造 gate 内候选 pair，按距离贪心选择一对一匹配。 |
| `update_track_from_cluster()` | 对匹配的真实 Track 做与第 10 步同样的 Kalman update。 |
| `update_track_lifecycle()` | 把预测、关联、更新、丢失、删除、新生按正确顺序串起来。 |
| `TrackLifecycleStats` | 统计本帧预测、匹配、丢失、新生和删除数量，用日志帮助排错。 |

第 06–10 步的 `debug_cv_state_` 仍保留，目的是让你继续看到教学日志；它不属于 `tracks_`，也不会占用真实 Track ID。现在看到的 `active_tracks` 才是实际 Track 数量。

## 新参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `max_missed_frames` | `5` | Track 连续未匹配多少帧后删除。只有 `missed_frames > 5` 才删除，因此会容忍 5 帧短暂丢失。 |
| `association_gate_m` | `0.40` | 沿用第 09 步，预测位置与 cluster 中心的最大匹配距离（米）。 |

在约 10 Hz 输入下，默认值容忍约 `0.5 s` 的无匹配时间。但这只是起始值：太小会让遮挡后频繁换 ID，太大则会长期保留已经不存在的 ghost Track。未来要用固定 rosbag、ID switch 和漏跟踪次数来调，而不是凭一次 RViz 画面决定。

## 构建

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

我已在当前工作区实际构建通过。以后你修改代码后，只要改的是这个包，也运行这一条构建命令。

## 如何运行和看日志

开三个终端；每个终端先执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

终端 1，启动动态场景：

```bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

终端 2，启动感知/聚类：

```bash
ros2 run predictive_nav_perception scan_info_node
```

终端 3，启动跟踪：

```bash
ros2 run predictive_nav_tracking tracking_node
```

启动时先看到若干类似日志是正常的：

```text
Track born | id=1 | cluster_index=0 | position=(...) m | velocity=(0.00, 0.00) m/s
Track born | id=2 | cluster_index=1 | position=(...) m | velocity=(0.00, 0.00) m/s
```

这不是“每帧都出错”，而是第一帧里多个 cluster 首次出现。之后每十帧会有摘要：

```text
track lifecycle | predicted=9 | matched=8 | missed=1 | born=0 | removed=0 | active=9
track[0] | id=1 | position=(...) m | velocity=(...) m/s | age=42 | missed=0
```

读法：

- `active=9`：当前内存中有 9 条真实 Track。它不一定等于“9 个动态人/方块”；当前 LiDAR cluster 也可能来自静态环境。
- `id=1`：这是跨帧稳定身份；它与 `cluster_index` 完全不同。
- `matched=8`：8 条旧 Track 找到了这一帧的合格观测并更新。
- `missed=1`：有 1 条旧 Track 这一帧没有匹配。一次 `missed` 很正常，不能立刻判死。
- `born=0`：本帧没有新出现且未匹配的 cluster。
- `age=42`：Track 已经历约 42 个有效处理帧；`missed=0` 表示刚刚成功匹配。

若连续丢失超出阈值，看到：

```text
Track expired | id=7 | age=38 frames | missed_frames=6 exceeds max_missed_frames=5
```

这说明 ID 7 已连续 6 帧没有可靠匹配，程序才删除它。它不是崩溃日志。

## 一个安全的可选观察

可以把容忍丢失帧数改小，方便更快看见生命周期现象：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args -p max_missed_frames:=2
```

这不会改代码或保存配置；结束节点后，不带参数重启就会回到默认 5 帧。请注意：**不要靠关闭感知节点来测试删除。** 感知节点关闭后，tracking 根本收不到新帧，自然也不能增加 `missed_frames`。真正的“丢失”是“仍收到新 cluster 帧，但某个旧 Track 没有合格关联”。

如果只想理解机制，不需要刻意制造删除场景；先确认 ID 会持续出现、`active_tracks` 不再总是 0 即可。

## 这一版还没有做什么

- 还没有 tentative / confirmed 两级 Track：目前一帧未匹配 cluster 就会出生，后续可用连续命中次数确认；
- 没有区分静态 Track 与动态 Track，`active_tracks` 只是被观察到的物体轨迹；
- 没有 Hungarian、Mahalanobis gate、外观特征或遮挡重识别；
- 还没有发布正式 `/dynamic_obstacles/tracks`，也还没有在 RViz 画 Track ID/速度箭头；这些是第 12、13 步。

## 本章必须懂的 ROS 2 / Nav2 知识

- Track 的时间依据依然是 cluster 消息的 `header.stamp`；没有有效 `dt` 时，不做真实生命周期推进。
- 所有关联位置必须在同一个 `odom` 坐标系；若收到其他 `frame_id`，节点只报警、不拿它和现有 Track 混算。
- `tracks_` 只是 tracking 节点内存中的状态。节点重启后 ID 会从 1 重新开始，这是当前阶段正常现象。
- 本步仍不发布 `/cmd_vel`，不会让小车因为 Track 出生/删除而自动转向或停车。

## 可选扩展知识

- tentative / confirmed Track 如何降低偶发误检造成的假目标；
- Mahalanobis distance 如何利用协方差让 gate 自适应；
- Hungarian 算法为什么能解决“一对一总距离最小”的分配问题；
- 多目标跟踪常见指标：ID switch、MOTA、MOTP、漏跟踪与误匹配。

## 完成检查

- [ ] `colcon build --packages-select predictive_nav_tracking` 成功。
- [ ] 启动后能看到一次或多次 `Track born`，且 ID 从 1 递增。
- [ ] 稳定后 `active_tracks` 不再恒为 0，并能看到 `track lifecycle` 摘要。
- [ ] 能解释 `cluster_index` 不是 ID，`track_id` 才是跨帧身份。
- [ ] 能解释为什么一次没匹配不能立即删除、为什么新 Track 初速度是 0。
- [ ] 知道当前一对一贪心匹配仍可能在交叉遮挡时换 ID。
