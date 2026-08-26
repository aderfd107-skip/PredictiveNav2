# 07：引入 CV 卡尔曼状态

## 这一小步为什么要做

第 06 步只是在两帧观测之间直接相减：这一帧有一个位置，上一帧也有一个位置，于是得到一个临时速度。它直观，但没有一个地方保存“我现在认为这个目标在哪里、朝哪里走、我有多确定”。

从这一小步开始，我们使用 **CV（Constant Velocity，匀速）模型**的状态：

```text
x = [px, py, vx, vy]ᵀ
```

`ᵀ` 可以暂时理解为“竖着写的向量”。四个数字的含义是：

| 下标 | 名称 | 单位 | 意义 |
|---:|---|---|---|
| 0 | `px` | m | 目标在 `odom` 中的 x 坐标 |
| 1 | `py` | m | 目标在 `odom` 中的 y 坐标 |
| 2 | `vx` | m/s | x 方向速度 |
| 3 | `vy` | m/s | y 方向速度 |

CV 不是说真实障碍物永远匀速，而是说：在相邻的短时间内，我们先用“它大致保持现有速度”这个简单假设。第 08 步会用它做预测；第 10 步才把新 LiDAR 测量融合进来，形成真正的 Kalman update。

## 协方差 `P` 又是什么

状态只有一个“最相信的数值”，但 LiDAR、聚类和模型都有误差，因此还要保留一个 4×4 矩阵：

```text
P = covariance（协方差）
```

此时你只要先记住两件事：

1. `P(i, i)` 是第 `i` 个状态量的不确定性大小，叫**方差**；越大表示越没有把握。
2. 方差 = 标准差 × 标准差。例如初始位置标准差 `0.20 m`，方差就是 `0.20² = 0.04 m²`。

所以本步默认初始化为：

```text
P_diag = [0.04, 0.04, 1.00, 1.00]
```

含义是：刚看见目标时，对位置大约有 0.20 m 的把握，但对速度还很不确定（标准差先设为 1.00 m/s）。非对角线先全部设为 0，表示此刻暂不假设“位置误差和速度误差具有已知关联”。以后 Kalman Filter 会自动产生这些关联。

## 本步写入的代码

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)

程序在首次拿到“有效 `dt` + 参考区域内的 cluster”时，创建一个仅供教学验证的 `debug_cv_state_`：

```cpp
debug_cv_state_.state << px, py, 0.0, 0.0;
debug_cv_state_.covariance = make_initial_cv_covariance();
```

此处初始 `vx=0`、`vy=0` 是一种谨慎的起点：只看到一次位置时，你还没有证据证明它朝哪个方向、以多快移动。第 06 步的两帧差分会继续作为对照输出，但本步不会把那种可能跳变的朴素速度直接写成可靠滤波状态。

`Eigen::Vector4d` 表示固定长度为 4 的 `double` 向量；`Eigen::Matrix4d` 表示 4×4 的 `double` 矩阵。选择固定尺寸是因为 CV 状态永远正好有 4 项，代码更清楚，也避免不必要的动态内存分配。

新增两个可调参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `initial_position_stddev_m` | `0.20` | 新状态的初始位置标准差（米） |
| `initial_velocity_stddev_mps` | `1.00` | 新状态的初始速度标准差（米/秒） |

## 注意：这还不是完整 Kalman Filter

本步没有：

- 创建真正进入 `tracks_` 的目标；
- 分配稳定的公开 track ID；
- 使用矩阵 `F(dt)` 预测下一时刻；
- 用 LiDAR 观测做 Kalman update；
- 解决第 06 步“选到了不同 cluster”的问题。

因此 `debug_cv_state_` 的 `track_id=0` 明确表示“教学调试状态”，不是将来发布给其他模块的真实 ID。它的作用只是让你现在就看到正确的状态和协方差数据结构。

## 构建与运行

在项目根目录：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

随后按第 06 步的三个终端启动动态仿真、感知节点和 tracking 节点。tracking 节点启动后，首次找到合格观测时会打印一次：

```text
CV state initialized (debug only) | x=[px=2.25, py=-2.86, vx=0.00, vy=0.00] | P_diag=[0.040, 0.040, 1.000, 1.000] | cluster_index=...
```

这行应这样读：

- `px`、`py` 来自该次 LiDAR cluster 中心；
- `vx=vy=0` 是初始化假设，不是在说方块真的静止；
- `P_diag` 中前两个 `0.040` 来自 `0.20²`；后两个 `1.000` 来自 `1.00²`；
- `cluster_index` 依然不是稳定 ID。

如果没有看到这一行，先检查第 06 步的日志是否持续有 `dt=... (valid)`，以及是否能在参考区域内找到 cluster。节点重启后会重新初始化一次，这是正常的。

## 一个可选观察实验

把位置标准差改成 0.50 m：

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args -p initial_position_stddev_m:=0.50
```

初始化日志中的前两项 `P_diag` 会变为 `0.250`，因为 `0.50² = 0.25`。这能帮助你确认：参数写的是“标准差”，矩阵里存的是“方差”。结束后不带参数重启即可恢复默认值。

## 本章必须懂的 ROS 2 / Nav2 知识

- 这个状态完全在节点内存中保存；当前没有新 topic，也没有控制小车。
- 位置、速度和 covariance 都必须基于同一个 `frame_id` 与时间语义；本项目短时 tracking 统一使用 `odom`。
- ROS 参数适合保存可实验调整的模型假设，例如初始不确定性；不要为了改一个数就反复改源码。

## 可选扩展知识

- “标准差”和“方差”的数学关系，以及为什么方差不能是负数。
- 协方差矩阵的非对角线为什么可以表示两个变量的相关性。
- 为什么 Kalman Filter 需要同时保存 `x` 和 `P`，而不只是一个位置和速度。

## 完成检查

- [ ] `colcon build --packages-select predictive_nav_tracking` 成功。
- [ ] 节点打印一次 `CV state initialized (debug only)`。
- [ ] 能解释 `x=[px, py, vx, vy]` 的四项和单位。
- [ ] 能解释 `0.20 m` 为什么在 `P_diag` 中变成 `0.040`。
- [ ] 明白本步尚未预测、更新或解决多目标关联。
