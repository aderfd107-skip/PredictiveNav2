# 11：写入动态风险评分

> **本步目标：** 把距离、TTC 与预测不确定性变成 MPPI 可以比较的非负动态风险代价。
>
> **AI 会做什么：** 生成可测试的风险函数、`data.costs` 累加逻辑和安全/冲突候选的最小测试。
>
> **你只需要做到：** 确认距离更小、时间更近时风险不会降低；确认无 prediction 时不会凭空增加代价。

把距离、时间和预测不确定性组合成非负风险代价，累加到 MPPI 每条候选轨迹的 cost。

**cost（代价）**只是“越大越不想选”的分数，不需要把它误解为真实物理能量。先采用可解释的平滑公式，再调参。

完成标准：危险候选的总 cost 比安全候选高，且无预测时不会凭空增加代价。

## 本步：让 AI 生成风险函数，你审查评分契约

这是第四章最核心的算法实现。让 AI 将“距离、尺寸、协方差、时间和 TTC”写成有上限、非负、可解释的 cost，并累加到对应候选的 `data.costs`。你要审查它是否遵守下方的性质和累加规则；这比逐行默写公式更重要。

## 需要的头文件

```cpp
#include <algorithm>
#include <cmath>
#include <limits>
```

## 建议让 AI 拆出的函数

```cpp
double max_eigenvalue_2x2(
  double covariance_xx_m2,
  double covariance_xy_m2,
  double covariance_yy_m2);

double safe_radius_m(
  double robot_radius_m,
  double obstacle_radius_m,
  double base_margin_m,
  double sigma_scale,
  double max_position_variance_m2);

double smooth_distance_risk(
  double distance_m,
  double safe_radius_m,
  double influence_radius_m,
  double collision_cost);

double ttc_risk(double ttc_s, double ttc_horizon_s);
```

不要把这四段都塞进 `score()`。可测试的小函数、清晰变量名和边界检查，比一段很长的三层循环更适合面试讲解。

## 你要先确认的性质

你的函数应满足：

```text
distance 越小      → risk 不减
协方差越大         → safe radius 不减
TTC 越小且为正     → ttc risk 不减
distance 足够远    → risk = 0
所有输入有限时     → 输出必须有限且 ≥ 0
无可用 prediction  → 不改变 data.costs
```

这六条比“公式看起来复杂”更重要；以后可直接写成 gtest。

## 写入 MPPI cost 时的规则

```text
只给当前 candidate 的 cost 加值
不能覆盖其他 Critic 已经写入的 cost
每次加值前后检查 finite
单轨迹/单时间点风险要有上限，防止一帧异常协方差让整个数组变 Inf
```

让 AI 生成一条安全候选和一条冲突候选的最小测试；你先判断谁的 cost 更高，再运行日志或测试验证该判断。

## 建议的第一版公式：简单、连续、可解释

第一版不追求论文级复杂公式。你先把每个候选点的有效安全半径写成：

```text
safe_radius = robot_radius + obstacle_radius + base_margin
              + sigma_scale × sqrt(max_eigenvalue(covariance_2x2))
```

然后定义距离风险的性质而非死背某个公式：

```text
d >= influence_radius                  → 0
d <= safe_radius                       → collision_cost（再加上限）
safe_radius < d < influence_radius     → 在 0 到 collision_cost 间连续上升
```

可先让 AI 选择线性、平方或指数衰减，并生成曲线或样例表；你需要能说明选它的理由。TTC 风险也必须有单独权重，不能与距离风险混成无法解释的常数。

## 写入 `data.costs` 的最小步骤

```text
for each candidate i:
  candidate_risk = 0
  for each usable time point / obstacle:
    candidate_risk += bounded distance risk + bounded TTC risk
  if finite(candidate_risk):
    data.costs(i) += dynamic_cost_weight × candidate_risk
```

不要在任意一个时刻检测到近距离就直接覆写 `data.costs(i)`；其他 Critic 的路径、静态障碍、平滑性评分都已存在。你只能累加自己的非负部分。

## 必做的最小测试

让 AI 根据下表生成测试数据和预期排序；你检查后运行它：

| 候选 | 最近距离 | 预测方差 | TTC | 预期风险 |
|---|---:|---:|---:|---|
| A：远离障碍 | 2.0 m | 小 | 无 | 0 或很小 |
| B：同一时刻擦边 | 0.30 m | 小 | 1.0 s | 高 |
| C：距离相同但预测不确定 | 0.30 m | 大 | 1.0 s | 不低于 B |

只有测试排序与日志排序一致，才进入第 12 节调权重。
