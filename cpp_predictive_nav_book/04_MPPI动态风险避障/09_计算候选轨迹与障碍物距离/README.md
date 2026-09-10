# 09：计算候选轨迹与障碍物距离

对 MPPI 的每条候选机器人轨迹，在每个时间点计算它与同一时刻障碍物预测位置的距离。

关键是“同一时刻”比较，而不是拿机器人现在的位置去比障碍物未来的位置。

完成标准：先输出距离诊断，不加代价；能发现一条会靠近障碍物的候选轨迹。

## 本步必须手写

这是动态规划核心的第一段数学代码：自己写候选轨迹、时间点、预测障碍物三层遍历，并先得到最小距离诊断。此步**不许**给 `data.costs` 加分。

## 要读懂的 MPPI 数据布局

本机 Jazzy 中：

```text
data.trajectories.x、y、yaws 的形状约为 [batch_index, time_index]
data.costs 的每一个元素对应一条 batch 候选
data.model_dt 是相邻机器人候选点的时间间隔
```

## 建议你亲自写的辅助函数

```cpp
double distance_2d(
  double robot_x_m, double robot_y_m,
  double obstacle_x_m, double obstacle_y_m);

std::size_t candidate_count(const mppi::CriticData & data);
std::size_t time_step_count(const mppi::CriticData & data);
```

`distance_2d()` 内部应使用 `std::hypot(dx, dy)`；先验证所有输入有限。

## 先写伪代码

```text
prediction = snapshot
若 prediction 不可用：return

minimum_distance = +∞
for candidate i:
  for robot time step k:
    robot_position = trajectories.x(i,k), trajectories.y(i,k)
    for each predicted obstacle:
      先临时取一个可比较的预测点
      distance = distance_2d(...)
      更新 global / candidate minimum_distance

节流打印最小距离、candidate index、time index、track id
```

第 10 步才实现严谨的时间插值/对齐；本步的目的只是让你看懂 `CriticData` 张量访问与循环结构。不要为了方便把 prediction 的未来末点拿来和所有机器人时间点比较。

## 必须检查

- `x`、`y` 的 batch/time 维度一致；
- 预测轨迹为空时安全跳过；
- 不访问 `poses[index]` 之前先保证 index 有效；
- 日志只输出最小值摘要，不能在 20 Hz 控制循环打印每一个距离。
