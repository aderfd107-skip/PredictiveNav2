# 10：加入时间对齐与 TTC 风险

> **本步目标：** 将 MPPI 的候选时刻与 prediction 的离散未来时刻对齐，并得到可解释的近碰时间（TTC）。
>
> **AI 会做什么：** 生成插值、TTC 计算和边界测试：预测范围外、时间戳倒退、相对速度接近零、即将碰撞。
>
> **你只需要做到：** 能解释“空间距离近”不必然危险而“同一时刻会相撞”才危险，并能用一个数值例子说明时间对齐的必要性。

把机器人候选点与 prediction 的时间偏移对齐，并加入 TTC（Time To Collision，预计碰撞时间）等近碰指标。

TTC 不是“是否已撞上”，而是按当前相对运动估计多久后可能碰撞；它用于让机器人更早减速。

完成标准：同样的空间距离下，时间更近的相遇得到更高风险。

## 本步：让 AI 实现，你审读时间语义

让 AI 把第 09 步的“几何上很近”升级成“在同一时间很近”，并生成插值与 TTC 的单元测试。你的任务不是从零手写循环，而是读懂并核对：查询时刻怎样得到、预测范围外为何拒绝、相对速度接近零时为何不能硬算 TTC。

## 需要的 C++ 工具

```cpp
#include <algorithm>
#include <cmath>
#include <optional>

#include "builtin_interfaces/msg/duration.hpp"
```

## 建议让 AI 拆出的函数

```cpp
struct PredictedPoint2D
{
  double time_s;
  double x_m;
  double y_m;
  double covariance_xx_m2;
  double covariance_xy_m2;
  double covariance_yy_m2;
};

std::optional<PredictedPoint2D> interpolate_prediction_at(
  const predictive_nav_msgs::msg::PredictedTrajectory & trajectory,
  double query_time_s);

std::optional<double> compute_ttc_s(
  double relative_x_m, double relative_y_m,
  double relative_vx_mps, double relative_vy_mps);
```

## 你要审读的数学边界

对于 MPPI 第 `k` 个候选点：

```text
query_time_s = (k + 1) × data.model_dt
```

在障碍物 `time_offsets` 中找包住 `query_time_s` 的相邻两点，线性插值得到该时刻位置与协方差；超出预测 horizon 时返回 `nullopt`，不要重复使用最后一点伪装成未来真值。

TTC 的前置条件：相对速度平方不能太小，且相对运动确实在接近。否则返回 `nullopt`，而不是除以接近 0 的数。

## 生成后必须能解释

- 为什么机器人 `model_dt` 不必等于 prediction 的 `0.2 s`？
- 为什么只按数组下标 `k` 对齐是不可靠的？
- TTC 和离散轨迹最小距离分别补足了什么信息？

## 推荐流程：分开生成、分开验证

先完成 `interpolate_prediction_at()`，通过后才写 TTC。插值函数应有四种清楚的返回情形：

```text
query < 第一个 offset            → nullopt（本项目预测从未来开始）
query 位于两个 offset 之间       → 线性插值
query 恰好命中一个 offset        → 直接取该点
query > 最后一个 offset          → nullopt（不假装知道更远未来）
```

让 AI 生成一个可执行的数值测试；你要核对这个例子：`0.2 s` 时位置 `x=1`，`0.4 s` 时位置 `x=3`，查询 `0.3 s` 时应得到 `x=2`。比例应为：

```text
alpha = (query_time - t0) / (t1 - t0)
value = (1 - alpha) * value0 + alpha * value1
```

协方差第一版也按相同 `alpha` 线性插值，并且每个结果都先检查 finite；后续可以升级模型，但不能在这里偷偷把 covariance 丢掉。

### TTC 的最小可解释版本

令 `p = obstacle_position - robot_position`，`v = obstacle_velocity - robot_velocity`。请检查生成代码先判断：

```text
||v||² 很小                    → 没有可靠 TTC，返回 nullopt
dot(p, v) >= 0                 → 两者没有接近，返回 nullopt
t_closest = -dot(p, v)/||v||²
t_closest <= 0 或超 horizon    → 返回 nullopt
否则                            → 返回 t_closest
```

这是“最接近时刻”的近似，不是带复杂几何碰撞模型的精确 TTC；README 和代码注释都应诚实说明这一点。

## 本节验收前的两个数值断言

- 障碍物正远离机器人：TTC 必须为空；
- 两者相向而行、距离 2 m、相对速度 1 m/s：TTC 应约为 2 s。

让 AI 将断言写进单元测试或可复现的测试程序；你运行并检查结果。不要仅凭 RViz 画面判断时间对齐正确。
