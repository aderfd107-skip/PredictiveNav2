# 09：计算候选轨迹与障碍物距离

> **本步目标：** 对每条机器人候选轨迹和每条障碍物预测轨迹，得到同一时刻、同一坐标系下的距离；此时先不写最终风险公式。
>
> **AI 会做什么：** 生成 MPPI 张量遍历、预测点匹配和距离计算，并加入最小距离调试输出。
>
> **你只需要做到：** 能指出距离计算使用的是同一时刻、同一坐标系下的两个位置，并能用日志检查最小距离是否合理。

对 MPPI 的每条候选机器人轨迹，在每个时间点计算它与同一时刻障碍物预测位置的距离。

关键是“同一时刻”比较，而不是拿机器人现在的位置去比障碍物未来的位置。

完成标准：先输出距离诊断，不加代价；能发现一条会靠近障碍物的候选轨迹。

## 本步 AI 生成与审读

这是动态规划核心的第一段数学代码：AI 生成候选轨迹、时间点、预测障碍物的三层遍历，并先输出最小距离诊断。此步**不许**给 `data.costs` 加分；你要审读每层循环的对象与索引。

## 要读懂的 MPPI 数据布局

本机 Jazzy 中：

```text
data.trajectories.x、y、yaws 的形状约为 [batch_index, time_index]
data.costs 的每一个元素对应一条 batch 候选
data.model_dt 是相邻机器人候选点的时间间隔
```

## 建议由 AI 拆出的辅助函数

```cpp
double distance_2d(
  double robot_x_m, double robot_y_m,
  double obstacle_x_m, double obstacle_y_m);

std::size_t candidate_count(const mppi::CriticData & data);
std::size_t time_step_count(const mppi::CriticData & data);
```

`distance_2d()` 内部应使用 `std::hypot(dx, dy)`；先验证所有输入有限。

## 先读伪代码

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

## 实施顺序：先理解二维张量，再生成三层循环

本机 Jazzy 的候选轨迹真的是：

```cpp
data.trajectories.x    // xtensor<float, 2>
data.trajectories.y    // xtensor<float, 2>
data.costs             // xtensor<float, 1>
```

让 AI 用一个小表格或日志示例标注：`x(i, k)` 的 `i` 是第几条候选，`k` 是这条候选的第几个未来时刻；`costs(i)` 只对应候选 `i`，没有时间维度。你要能看懂这个对应关系。

然后分三次生成并验证：

1. 写 `distance_2d()`，并给它手算三个输入：`(0,0)-(3,4)` 应为 `5`；
2. 只读取 `shape()[0]`、`shape()[1]`，确认与 `data.costs.shape()[0]` 的关系；
3. 最后才写 candidate → time step → obstacle 的循环，并只记录全局最小距离摘要。

### 本节允许的临时比较规则

为了把张量访问和消息遍历跑通，可临时用每条障碍物预测轨迹的**第一个**预测点与 `k=0` 的机器人候选点比较，并在日志明确写 `diagnostic_only`。完成验证后马上删除这个临时限制；第 10 节必须替换为真正的时间查询。

### 不要犯的三个错误

- 以为 `data.costs(i, k)` 存在；它只是一维；
- 没检查 `x/y` shape 一致就索引；
- 把 `track_id` 当作 trajectory array 下标。

本节完成时，提供 AI 生成的 `distance_2d()`、循环摘要和一次最小距离日志即可。还不能修改 `data.costs`。
