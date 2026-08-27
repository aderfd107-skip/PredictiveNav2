# 08：预测已有轨迹

## 先用一句人话说明

假设你上一秒看到一个人正以每秒 0.3 米向北走，而这一瞬间被墙挡住了。即使暂时看不见他，你仍可以做一个短时间猜测：**0.1 秒后，他大约会再向北走 0.03 米。**

这就是本章的“预测”。它不是读取新的激光点，也不是修改小车速度；它只是把已有障碍物状态按时间往前推一步，为下一章判断“新看到的 cluster 是否可能还是它”准备参考位置。

## 先认识三个名词

| 名词 | 现在把它理解成什么 |
|---|---|
| 预测（predict） | 暂时不看新测量，只根据之前的位置、速度和经过时间估计现在在哪里 |
| CV 模型 | Constant Velocity，短时间内假设速度不变 |
| 过程噪声（process noise） | 目标可能加速、减速或模型不准带来的额外不确定性 |

第 07 步创建了状态卡：

```text
x = [px, py, vx, vy]ᵀ
```

其中 `px`、`py` 是位置，`vx`、`vy` 是速度。现在只要经过 `dt` 秒，就有最直观的 CV 预测：

```text
px_new = px_old + vx × dt
py_new = py_old + vy × dt
vx_new = vx_old
vy_new = vy_old
```

例如 `py=-2.86 m`、`vy=0.30 m/s`、`dt=0.10 s`，预测后 `py=-2.83 m`。注意正负号由坐标方向决定。

## 这一章实际写了什么

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)

程序新增两件事：

1. 用 `F(dt)` 把 CV 状态向前推：`x_new = F × x_old`。
2. 用 `P_new = F × P × Fᵀ + Q` 更新协方差。

不用急着背矩阵。你只要把第一条理解成上面的“位置加速度乘时间”；第二条的白话含义是：**预测时间越长、模型越不确定，就越不能自信。**

`Q` 就是过程噪声矩阵。这里假设障碍物可能有未知加速度；因此即使速度数值暂时不变，位置和速度的 `P_diag` 也会逐渐增加。

为了避免第 06 步出现的 `4 m/s`、`13 m/s` 这类 cluster 跳变直接污染演示状态，本步只接受速度大小不超过 `debug_max_initial_speed_mps`（默认 `0.80 m/s`）的连续观测来初始化教学状态。仿真方块约 `0.35 m/s`，通常能通过；离谱值会被忽略并继续等待。

这仍只是受控教学实验：`debug_cv_state_` 的 ID 固定为 0，不进入真正的 `tracks_` 数组，也没有稳定 ID、数据关联或 Kalman 测量更新。

## 新增参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `debug_max_initial_speed_mps` | `0.80` | 允许拿来初始化教学状态的最大朴素速度 |
| `process_acceleration_stddev_mps2` | `1.00` | 对未知加速度的标准差假设；越大，协方差增长越快 |

这些参数不是让方块真的变快或变慢；它们只影响程序是否接受初始速度、以及预测有多保守。

## 构建与运行

先构建：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

然后沿用第 06 步的三个终端：

```bash
# 终端 1
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true

# 终端 2
ros2 run predictive_nav_perception scan_info_node

# 终端 3
ros2 run predictive_nav_tracking tracking_node
```

先会出现一次初始化日志：

```text
CV state initialized (debug only) | x=[px=2.25, py=-2.86, vx=0.01, vy=0.30] | ... | seed_speed=0.30 m/s
```

随后每十帧会出现预测日志：

```text
CV predict (debug only) | dt=0.100 s |
before=(2.25, -2.86, 0.01, 0.30) |
after=(2.25, -2.83, 0.01, 0.30) |
P_diag=(...)
```

读法：

- `before` / `after` 的顺序始终是 `(px, py, vx, vy)`。
- `py` 每帧约改变 `0.30 × 0.10 = 0.03 m`；`vx`、`vy` 保持相同，这正是 CV 假设。
- `P_diag` 是协方差对角线，通常会逐步变大；这不是 bug，而是“只预测、不测量时会越来越不确定”。
- 若暂时没有 `CV state initialized`，说明还没有出现速度合理的连续观测；请等待，或确认动态 actor 和感知节点正在运行。

## 一个可选小实验

让过程噪声更大：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args \
  -p process_acceleration_stddev_mps2:=2.0
```

位置预测的公式不变，但 `P_diag` 会比默认值增长更快。这表示程序认为障碍物更可能突然改变速度，因此对远一点的预测更不自信。

## 本章故意还没有做什么

- 没有使用当前帧 cluster 修正预测位置；这会在第 10 步的 Kalman update 完成。
- 没有将多个 cluster 对应到多个 track；这会从第 09 步的数据关联开始。
- 没有删除长期未观察到的轨迹；这是第 11 步的生命周期管理。
- 没有发布正式 `/dynamic_obstacles/tracks`，也没有影响 Nav2 或 `/cmd_vel`。

因此，预测会持续沿着初始速度走，最终可能偏离真实方块；这不是最终效果，而是下一步必须拿“预测位置”作为匹配参考、再用新观测纠正的原因。

## 本章必须懂的 ROS 2 / Nav2 知识

- 真实 `dt` 来自消息 `header.stamp`，不能写死为 0.1 秒。
- 节点内部状态可以在没有新测量结果时继续预测，但必须记录何时会过期；第 11 步会正式处理生命周期。
- 此处使用的仍是 `odom` 坐标系，避免 AMCL 的地图校正被误当作障碍物运动。

## 完成检查

- [ ] `predictive_nav_tracking` 构建成功。
- [ ] 能看到一次 `CV state initialized`，其速度不超过 `0.80 m/s`。
- [ ] 能看到 `CV predict`，并解释 `before` 到 `after` 的位置变化。
- [ ] 知道 `P_diag` 增长表示预测越来越不确定。
- [ ] 明白本步还没有进行测量更新或多目标关联。
