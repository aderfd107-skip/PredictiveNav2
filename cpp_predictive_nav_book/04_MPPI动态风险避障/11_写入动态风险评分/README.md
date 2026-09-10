# 11：写入动态风险评分

把距离、时间和预测不确定性组合成非负风险代价，累加到 MPPI 每条候选轨迹的 cost。

**cost（代价）**只是“越大越不想选”的分数，不需要把它误解为真实物理能量。先采用可解释的平滑公式，再调参。

完成标准：危险候选的总 cost 比安全候选高，且无预测时不会凭空增加代价。

## 本步必须手写

这是第四章最核心的算法实现。你要自己把“距离、尺寸、协方差、时间和 TTC”变成有上限、非负、可解释的 cost，并累加到对应候选的 `data.costs`。

## 需要的头文件

```cpp
#include <algorithm>
#include <cmath>
#include <limits>
```

## 建议你自己拆出的函数

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

## 你要先写下的性质

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

提交代码前请手画一条安全候选和一条冲突候选，算出它们应当谁的 cost 更高，再让日志验证这个判断。
