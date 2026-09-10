# 08：检查预测时效与坐标系

在评分前检查 prediction 是否过期、frame 是否为 `odom`、点和协方差是否有限；不合格数据不能参与评分。

**时效**是“预测距现在多久”。动态环境里一份旧预测很可能已经指向错误位置。

完成标准：停止 prediction 节点后，critic 明确报告过期并执行配置好的安全策略。

## 本步必须手写

你要把“收到过消息”与“现在还能用这条消息”严格分开。旧 prediction 参与控制，比没有 prediction 更危险。

## 需要的头文件与成员

```cpp
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
```

```cpp
bool is_prediction_usable(
  const PredictionArray & message,
  const rclcpp::Time & now,
  std::string * reason) const;

double max_prediction_age_s_{0.5};
std::string expected_prediction_frame_{"odom"};
```

## 你要自己完成的检查顺序

```text
1. header.frame_id 是否等于 odom
2. header.stamp 是否为正、可转换为 ROS 时间
3. now - stamp 是否在 [允许的轻微 future skew, max_prediction_age] 内
4. trajectories / poses / time_offsets 数量是否匹配
5. 每个位置、尺寸、confidence、协方差元素是否 finite
6. time_offsets 是否严格递增且为正
```

`std::isfinite()` 用于拒绝 `NaN` 与 `Inf`。不要用 `value == NAN`；这是 C++ 常见错误。

## 安全策略边界

本步先把策略参数化为 `skip_dynamic_risk`：prediction 不可用时不使用旧缓存，同时节流告警。Critic 不能自己发布停车命令；后面会讨论“高 cost / fail flag / 外部安全层”的组合，不能在这里偷偷创建 `/cmd_vel` publisher。

## 你的自测题

- prediction_node 被 Ctrl+C 停掉后，为什么 `/clock` 继续走才会让 age 变大？
- frame 是 `map` 但数值看着正常，为什么仍必须拒绝？
- 空 trajectory array 与 prediction node 已死，为什么是两个不同状态？
