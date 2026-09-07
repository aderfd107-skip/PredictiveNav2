# 06：CV 传播未来位置

> 本步目标：对第 02 模块给出的每条 Track，按照第 05 步的未来时间轴计算位置预测。现在只在终端预览计算结果；协方差传播和正式 prediction 消息发布留到后续步骤。

## 1. CV 到底在算什么？

CV 是 **Constant Velocity（匀速）** 的缩写。它做的假设是：在短时间内，障碍物继续保持当前的 x/y 速度。

```text
现在位置：       (x_now, y_now)
现在速度：       (vx, vy)
未来时间偏移：   t

x_future = x_now + vx × t
y_future = y_now + vy × t
```

例如 Track 当前为：

```text
position = (2.00, 1.00) m
velocity = (0.50, -0.20) m/s
```

则：

| offset | x 计算 | y 计算 | 预测位置 |
| ---: | --- | --- | --- |
| 0.2 s | `2.00 + 0.50 × 0.2` | `1.00 + (-0.20) × 0.2` | `(2.10, 0.96) m` |
| 1.0 s | `2.00 + 0.50 × 1.0` | `1.00 + (-0.20) × 1.0` | `(2.50, 0.80) m` |
| 2.0 s | `2.00 + 0.50 × 2.0` | `1.00 + (-0.20) × 2.0` | `(3.00, 0.60) m` |

可以看到：未来点会沿 `(vx, vy)` 所指的方向排列。

## 2. “传播”不是重新测量，也不会让方块真的这样走

这里最容易混淆：

```text
LiDAR / Track：这是传感器在当前时刻观测或估计到的状态。
CV prediction：这是程序基于当前状态作出的未来假设。
```

CV 不会向 Gazebo 发命令，也不会让真实障碍物按预测轨迹移动。下一个真实观测到达后，tracking 会重新估计速度和位置，prediction 又会从新的当前状态重新向未来推。

所以每一帧都是：

```text
新的 Track 当前状态
      ↓
重新从 t=0 向未来计算 0.2、0.4、...、2.0 s
      ↓
旧预测自然被新预测替换
```

## 3. 代码做了什么？

新增了：

```cpp
std::vector<PredictedPosition2D> propagate_cv_positions(
  const TrackedObstacle & track,
  const std::vector<double> & time_offsets_s);
```

### `PredictedPosition2D` 是什么？

```cpp
struct PredictedPosition2D
{
  double time_offset_s;
  double x_m;
  double y_m;
};
```

它是这个步骤内部使用的小结构体，保存一个未来点的三件事：未来多久、未来 x、未来 y。它不是 ROS 消息；第 09 步才会把结果填到 `PredictedTrajectory` 消息中。

### 为什么传入 `const TrackedObstacle &`？

```cpp
const TrackedObstacle & track
```

- `&`：引用，不复制整条 Track，效率更好；
- `const`：函数只读取 Track，不能修改 tracking 模块传来的原始数据。

这很符合模块边界：prediction 可以“读当前状态、产生自己的预测”，不能倒回去偷偷修改 Track。

### 先检查有限数值

代码会检查 x、y、vx、vy 是否为有限数。如果遇到 `NaN` 或 `Inf`，此步骤不产生预览结果。第 08 步会把这种异常、过期时间和 frame 问题做成完整的拒绝策略；现在先避免错误数值继续参与乘法。

## 4. 为什么只显示一条 Track 的 CV 预览？

真实 `/tracks` 常常包含墙面、桌椅等静态物体。默认 10 个未来点 × 多条 Track 会把终端刷得无法阅读。

因此每一帧：

1. 程序对所有 Track 都具备调用 CV 函数的能力；
2. 仅挑选当前速度最大的 Track 作为**调试展示对象**；
3. 日志展示它的第一个、中间和最后一个未来点。

这不是“只预测速度最快的目标”的算法规则。第 09 步正式发布时，每条有效 Track 都会获得完整的预测轨迹。

若静态物体被 tracking 噪声误估出较大速度，它也可能被选作预览对象。这反而是一个有价值的观察：它说明 Track 的速度或关联质量仍需后续评估，而不是 CV 公式算错。

## 5. 如何运行

保持仿真、感知、tracking 节点运行，然后在 prediction 终端执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node
```

预期先出现时间轴日志，再每秒出现类似：

```text
CV preview | id=20 | now=(2.00, 1.00) m | vel=(0.50, -0.20) m/s |
t=0.20 s -> (2.10, 0.96) m |
t=1.00 s -> (2.50, 0.80) m |
t=2.00 s -> (3.00, 0.60) m
```

你应检查：

- 若 `vx > 0`，未来 x 应逐渐增大；若 `vx < 0`，未来 x 应逐渐减小；
- 若 `vy > 0`，未来 y 应逐渐增大；若 `vy < 0`，未来 y 应逐渐减小；
- 速度接近 0 时，未来位置应接近当前位置；
- 所有位置仍是 `odom` 坐标系中的米，速度仍是 `m/s`。

## 6. 常见疑问与排错

### 预览出来的速度很小，位置几乎不变，正常吗？

正常。它说明这条 Track 当前估计接近静止。CV 的正确行为就是预测它继续停在附近，而不是凭空制造运动。

### 为什么预览对象看起来是墙，而不是 Gazebo 动态方块？

当前只是选择“速度数值最大”的一条作日志展示；静态激光 cluster 的抖动、关联误差可能造成假速度。请先把日志截图发来，我们判断是 tracking 质量问题、ID 切换，还是动态方块在当前 LiDAR 视野外。正式算法后续会有置信度、协方差、过期检查和风险门控，不能只凭一条速度决定风险。

### CV 会不会预测错？

会。方块转弯、突然停下、被遮挡、速度估计错误时，CV 的未来位置会偏离真实运动。因此：

- 第 07 步传播不确定性，预测越远越不自信；
- 第 08 步拒绝过期或异常 Track；
- 第 04 模块不会把预测当真值，而是把协方差转成更保守的风险范围；
- 真机阶段会用 rosbag 与统计指标量化 CV 在哪些情景失效。

## 7. 本步完成标准

- [ ] 构建成功，节点仍能订阅 `/dynamic_obstacles/tracks`；
- [ ] 每秒看到一条 `CV preview`；
- [ ] 能用 `x_future=x+vx×t`、`y_future=y+vy×t` 手算并核对一项日志；
- [ ] 知道当前只是位置均值预测，尚未处理未来不确定性；
- [ ] 知道此时没有发布 `/dynamic_obstacles/predictions`，也没有影响 Nav2 控制。

## 本章必须懂的 ROS 2 / C++ 知识

- CV 是短时间运动假设，不是传感器读数，也不是控制命令。
- `const T &` 表示以只读引用传入对象，避免不必要的复制。
- `std::vector` 可以保存任意自定义结构体，例如一串 `PredictedPosition2D`。
- 高速终端日志应只显示代表性结果；调试展示范围不等于算法实际处理范围。

## 下一步

[07_传播位置不确定性](../07_传播位置不确定性/README.md)：未来位置均值已经能计算，但“预测到 2 秒后有多不可靠”还没表示。下一步把 Track 的协方差传播到每个未来点。
