# 07：传播位置不确定性

## 本步完成后，你得到了什么

第 06 步只回答了“按当前速度推算，目标**可能在哪里**”。但它把预测位置当成一个精确点，这是不真实的：速度有估计误差，目标也可能突然加速、减速或转弯。

本步让每一个未来预测点同时带有一个二维位置协方差 `P_position`。它表示：预测得越远，我们对位置越没有把握。当前仍只做终端预览；还不会发布 `/predictions`，更不会控制 Nav2。

## 先认识：什么叫协方差

对于二维位置 `(x, y)`，位置协方差是一个 2×2 矩阵：

```text
P_position = [ Pxx  Pxy ]
             [ Pyx  Pyy ]
```

- `Pxx`：x 方向位置误差的方差，单位是 `m²`。
- `Pyy`：y 方向位置误差的方差。
- `Pxy` / `Pyx`：x、y 两个误差是否相关；数学上协方差矩阵应当对称，所以 `Pxy = Pyx`。

方差不是“坐标偏了多少米”，而是误差范围的平方量。`Pxx` 越大，表示 x 位置越不可靠；将来在 RViz 中可以把它画成越来越大的不确定性椭圆。

## 一个关键接口修正：为什么新增 `state_covariance[16]`

跟踪器内部的 CV 卡尔曼状态是：

```text
x = [px, py, vx, vy]^T
```

其完整协方差 `P` 是 4×4。之前消息里虽然有 `pose.covariance` 和 `twist.covariance`，但它们分别保存位置块、速度块，丢掉了“位置和速度之间的交叉协方差”。而严格地传播未来位置时，这部分信息不能丢。

因此本步给 `TrackedObstacle.msg` 增加：

```text
float64[16] state_covariance
```

它按照**行优先**保存完整 4×4 矩阵，状态顺序固定为 `[px, py, vx, vy]`：

```text
index:  0   1   2   3
row 0: P(px,px) P(px,py) P(px,vx) P(px,vy)
row 1: P(py,px) P(py,py) P(py,vx) P(py,vy)
row 2: P(vx,px) P(vx,py) P(vx,vx) P(vx,vy)
row 3: P(vy,px) P(vy,py) P(vy,vx) P(vy,vy)
```

记住最重要的一点即可：这个数组是跟踪器完整内部 `P` 的副本，不是随意拼出来的 16 个数。

## 本步使用的公式

CV（匀速）模型下，未来位置均值还是第 06 步的：

```text
x(t) = x0 + vx × t
y(t) = y0 + vy × t
```

位置协方差则按下面的式子增长：

```text
P_position(t) = Ppp + t(Ppv + Pvp) + t²Pvv + Q_position(t)

Q_position(t) = (t⁴ / 4) × σa² × I
```

可以这样理解：

- `Ppp`：当前已经存在的位置不确定性。
- `Pvv`：速度不确定性；时间越久，速度误差累积得越明显，所以乘 `t²`。
- `Ppv`、`Pvp`：位置与速度的不确定性关联，这正是完整 4×4 `P` 的价值。
- `Q_position`：模型承认目标可能有未建模的加速度。`σa` 越大，远期预测越保守；`I` 表示默认 x、y 的未知加速度相互独立。

这里的 `P_future = F P F^T + Q` 是卡尔曼预测的协方差形式；上面的公式只是把它取出了“位置的 2×2 部分”。

## 新参数：未知加速度标准差

预测节点新增参数：

```text
prediction_acceleration_stddev_mps2 = 1.0
```

单位是 `m/s²`。它不是测得的真实加速度，而是模型对“目标可能不再匀速”的保守程度。

- 调大，例如 `2.0`：不确定性随时间增长更快，后续避障会更保守。
- 调小，例如 `0.3`：更相信匀速模型，但对突然转弯的人或物体可能过度自信。
- 设为 `0.0`：只传播已有状态误差；适合理解公式，不适合真实动态目标。

当前先保持默认 `1.0`，以后通过 rosbag 和真机数据再调参，而不是凭感觉确定最终数值。

## 代码中做了什么

`prediction_node.cpp` 的 `propagate_cv_positions(...)` 对代表性的一个 Track（当前速度最大的 Track，仅用于减少日志）执行以下事情：

1. 检查位置、速度、参数和 16 个协方差值都不是 NaN/无穷大。
2. 对时间轴的每个 `t`，按 CV 公式计算未来均值位置。
3. 用完整 `state_covariance` 计算 `Pxx`、`Pxy`、`Pyy`。
4. 将 `Pxy` 与 `Pyx` 平均，消除浮点计算造成的极小不对称。
5. 在日志中打印三个时刻的位置与 `Pdiag=(Pxx, Pyy)`。

`Pdiag` 的单位为 `m²`。一般来说，越远的时刻数值越大；但若当前位置—速度交叉协方差为负，短时间内不一定严格单调，这是卡尔曼滤波中正常的数学现象。不要只用“是否单调”判断代码对错。

## 构建与运行

修改了自定义消息，必须连同消息包、跟踪包和预测包一起编译：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to predictive_nav_tracking predictive_nav_prediction
source install/setup.bash
```

之后按以前的方式启动仿真、感知、跟踪和预测节点。预测节点启动时应看到：

```text
prediction process model | acceleration_stddev=1.00 m/s^2
prediction_node started. Step 07 previews CV positions and position covariance only.
```

有 Track 输入时，预期日志形态如下：

```text
CV preview | id=... | now=(...) m | vel=(...) m/s |
t=0.20 s -> (...) m, Pdiag=(..., ...) m^2 | ...
t=2.00 s -> (...) m, Pdiag=(..., ...) m^2
```

截图时重点看远期 `Pdiag` 是否通常比近期大；静态墙体、误关联出来的 Track 或刚出生的 Track 都可能让数值不具备直观性，这正是下一步要处理数据有效性的原因。

## 本章必须懂的 ROS 2/Nav2 知识

- 自定义 `.msg` 改动后，订阅或发布它的所有 C++ 包都必须重新编译并重新 `source install/setup.bash`。
- ROS 的 `PoseWithCovariance` / `TwistWithCovariance` 是 6×6 表达，适合通用三维接口；本项目的内部平面 CV 状态是 4×4，单独发布 `state_covariance` 能保留算法所需的信息。
- 预测模块目前只是观察 `/dynamic_obstacles/tracks`；它不替代 Nav2 的 controller，也没有向 `/cmd_vel` 发布任何消息。

## 可选扩展知识

- 用特征值和特征向量将 2×2 协方差画成 RViz 不确定性椭圆。
- 把 `σa` 设为不同值，使用同一段 rosbag 比较“过度自信”和“过度保守”的差异。
- 学习离散白噪声加速度模型的完整过程噪声矩阵 `Q`。

## 下一步

第 08 步将处理无效、过旧或刚出生的 Track，避免把不可靠状态直接作为正式预测结果发布。
