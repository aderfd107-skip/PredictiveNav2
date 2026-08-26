# 06：单目标连续观测与朴素速度

## 这一小步到底在做什么

上一小步确认了每一帧 cluster 都带有可靠时间戳，因此现在可以回答一个最基本的问题：**同一个障碍物在两帧之间移动了多少？**

本步先不急着做“多目标跟踪”。我们只在仿真场景下指定下方动态方块会经过的一小块区域，持续从真实的 LiDAR 聚类结果中选出离该区域最近的一个 cluster，然后用相邻两帧的中心点直接相减，得到速度。

```text
              当前帧位置 - 上一帧位置
速度 =  --------------------------------
                       dt

vx = (x_now - x_previous) / dt
vy = (y_now - y_previous) / dt
```

这里的 `x`、`y` 单位是米，`dt` 单位是秒，所以 `vx`、`vy` 的单位自然就是 `m/s`（米每秒）。

本步的产物只是终端调试输出，不创建真正的 `Track`，不分配 ID，不发布 `/dynamic_obstacles/tracks`，更不会影响小车导航。它的价值是把后续 Kalman Filter 中“速度从哪里来”变得看得见、摸得着。

## 很重要：这不是读取 Gazebo 真值

代码唯一订阅的话题仍然是：

```text
/dynamic_obstacles/clusters
```

它来自本项目的 `/scan → 聚类` 链路。程序**没有订阅** Gazebo 的真值 Marker，也没有直接读取方块坐标。

为了做一个可控的单目标实验，程序有一个“参考位置” `(2.25, -2.85)`：它只是告诉程序“在这一片区域附近，从已经检测到的所有 cluster 中选一个最近的”。若最近 cluster 仍超过 1 米，就认为这一帧没有找到目标。这样你能先专心理解速度公式；真正不依赖人工区域的多目标关联会在第 09 步开始实现。

## 代码新增了什么

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)

新增的三个 ROS 参数如下：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `debug_target_x_m` | `2.25` | 单目标实验区域的 x 坐标（`odom`，米） |
| `debug_target_y_m` | `-2.85` | 单目标实验区域的 y 坐标（`odom`，米） |
| `debug_target_max_distance_m` | `1.00` | 接受最近 cluster 的最大距离（米） |

程序每收到一帧 cluster：

1. 像第 05 步一样从 header 计算并验证 `dt`。
2. 遍历这一帧的全部 cluster，算它们到参考位置的二维距离。
3. 保留最近且距离不超过阈值的 cluster。
4. 第一次有效观测只记住中心点，输出 `velocity=warming_up`；因为只有一个位置，不能算速度。
5. 从第二次连续有效观测开始，计算 `vx`、`vy` 和总速度 `speed`。
6. 若 `dt` 不合法、目标不见了，程序清空上一帧记录；下一次重新找到目标后会再次 `warming_up`，不会错误地跨很长时间直接相减。

注意：这里故意**没有**写 `message->clusters[0]`。cluster 数组的第 0 个元素不保证永远是同一物体；直接拿下标做跨帧跟踪是一个常见但错误的初学者做法。

## 先构建

在项目根目录执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

看到 `Finished <<< predictive_nav_tracking` 就说明 C++ 代码已编译成功。

## 如何运行和观察

请开三个终端。每个终端都先执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

终端 1，启动带动态障碍物的仿真与导航：

```bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

终端 2，启动第 01 章完成的感知节点：

```bash
ros2 run predictive_nav_perception scan_info_node
```

终端 3，启动本章 tracking 节点：

```bash
ros2 run predictive_nav_tracking tracking_node
```

稳定后每十帧会有一组类似输出：

```text
cluster frame #120 | frame=odom | ... | dt=0.100 s (valid) | active_tracks=0 ...
single-target debug | cluster_index=2 | centroid=(2.24, -2.61) m | reference_distance=0.24 m | naive_velocity=(0.01, 0.33) m/s | speed=0.33 m/s
```

读法：

- `dt=0.100 s (valid)`：两帧相隔约 0.1 秒，可以安全计算速度。
- `centroid=(...)`：这个 cluster 的 LiDAR 估计中心，坐标系是 `odom`。
- `cluster_index=2`：它在**这一帧**数组里的位置。它可能变，不能当 ID。
- `naive_velocity=(0.01, 0.33) m/s`：x 方向几乎不动，y 方向约每秒移动 0.33 米，符合下方方块沿 y 轴往返的运动。
- `speed=0.33 m/s`：速度大小，等于 `sqrt(vx² + vy²)`。

方块走到端点并折返时，`vy` 正负号会改变；这是正常现象。输出不需要每一行恰好是 `0.35`，激光离散、聚类中心抖动和 0.1 秒差分都会带来波动。

若出现：

```text
single-target debug | ... | velocity=warming_up
```

表示程序刚开始记录位置，下一帧连续有效观测才有速度。若出现：

```text
no cluster within 1.00 m
```

表示当前在这块区域附近没有合格 cluster；确认动态 actor 已启用、感知节点正在运行，或等待方块进入这条路段。

## 一个安全的小实验（可选）

把最大可接受距离故意改小到 0.25 m：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args -p debug_target_max_distance_m:=0.25
```

你会更常看到 `no cluster within ...` 和之后的 `warming_up`。这证明“目标丢失后不拿旧位置继续算速度”的保护逻辑在工作。实验结束后按 `Ctrl+C`，不带参数重新运行即可恢复默认值。

## 这一步的局限，正是后续要解决的问题

朴素差分非常直观，但还不够当作机器人最终决策依据：

- LiDAR 和聚类中心有抖动，除以很小的 `dt` 后速度会更抖。
- 参考区域是人为指定的；多个目标靠近时它会选错。
- 它没有 ID、没有协方差，也没有“这个目标暂时被遮挡了怎么办”的能力。
- cluster 下标会变，因此不能把 `cluster_index` 当作目标编号。

下一步会用 Constant Velocity（CV）模型和 Kalman Filter 的状态向量 `[px, py, vx, vy]`，把这里看见的“位置与速度”变成一个可预测、可更新的数学状态；之后再做真正的多目标关联。

## 本章必须懂的 ROS 2 / Nav2 知识

- topic 的一条数组消息只表示某个时刻的观测；跨 callback 保存的数据才构成“历史”。
- `header.stamp` 是速度计算的时间依据，不应假设 callback 到达间隔恒定。
- 本章依旧在 `odom` 坐标系中计算速度，避免 AMCL 的 `map → odom` 修正被误判成障碍物本身运动。
- 参数可以在启动节点时用 `--ros-args -p 参数名:=值` 临时覆盖，不需要改代码。

## 可选扩展知识

- 中心差分、滑动窗口平均为何比两帧直接相减更平滑。
- data association（数据关联）如何在没有人工参考区域时判断“哪一个观测属于哪一条轨迹”。
- Kalman Filter 如何同时保存均值和不确定性，而不是只保存一个速度数字。

## 完成检查

- [ ] `predictive_nav_tracking` 构建成功。
- [ ] tracking 终端持续显示 `dt≈0.100 s (valid)`。
- [ ] 能看到 `single-target debug` 的 `centroid` 与 `naive_velocity`。
- [ ] 理解第一帧为什么只能显示 `warming_up`。
- [ ] 理解这不是完整多目标跟踪，也没有使用 Gazebo 真值。
