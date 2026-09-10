# 10：加入时间对齐与 TTC 风险

把机器人候选点与 prediction 的时间偏移对齐，并加入 TTC（Time To Collision，预计碰撞时间）等近碰指标。

TTC 不是“是否已撞上”，而是按当前相对运动估计多久后可能碰撞；它用于让机器人更早减速。

完成标准：同样的空间距离下，时间更近的相遇得到更高风险。

## 本步必须手写

本步把第 09 步的“几何上很近”升级成“在同一时间很近”。时间对齐、插值与 TTC 是本项目最值得你亲自写和解释的部分。

## 需要的 C++ 工具

```cpp
#include <algorithm>
#include <cmath>
#include <optional>

#include "builtin_interfaces/msg/duration.hpp"
```

## 建议你自己设计的函数

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

## 你要实现的数学边界

对于 MPPI 第 `k` 个候选点：

```text
query_time_s = (k + 1) × data.model_dt
```

在障碍物 `time_offsets` 中找包住 `query_time_s` 的相邻两点，线性插值得到该时刻位置与协方差；超出预测 horizon 时返回 `nullopt`，不要重复使用最后一点伪装成未来真值。

TTC 的前置条件：相对速度平方不能太小，且相对运动确实在接近。否则返回 `nullopt`，而不是除以接近 0 的数。

## 你写完后必须能解释

- 为什么机器人 `model_dt` 不必等于 prediction 的 `0.2 s`？
- 为什么只按数组下标 `k` 对齐是不可靠的？
- TTC 和离散轨迹最小距离分别补足了什么信息？
