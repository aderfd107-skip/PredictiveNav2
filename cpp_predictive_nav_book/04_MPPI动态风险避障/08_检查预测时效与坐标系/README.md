# 08：检查预测时效与坐标系

> **本步目标：** 在风险评分前拒绝过期、frame 不一致或包含异常数值的 prediction；此时仍不计算距离。
>
> **AI 会做什么：** 生成时效、frame 和数组合法性检查，以及过期阈值、拒绝日志和最小测试。
>
> **你只需要做到：** 能解释为什么同一个数值在 `map` 与 `odom` 中不能直接相减，并演示一条过期或 frame 错误消息被安全忽略。

在评分前检查 prediction 是否过期、frame 是否为 `odom`、点和协方差是否有限；不合格数据不能参与评分。

**时效**是“预测距现在多久”。动态环境里一份旧预测很可能已经指向错误位置。

完成标准：停止 prediction 节点后，critic 明确报告过期并执行配置好的安全策略。

## 本步 AI 生成与审读

AI 会把“收到过消息”与“现在还能用这条消息”严格分开。你要读懂为什么旧 prediction 参与控制比没有 prediction 更危险，并通过停掉 prediction 节点验证安全退化。

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

## 你要审读的检查顺序

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

## 先读真实消息，再写校验函数

本项目的 `PredictedTrajectoryArray` 有一个公共 `header` 和多个 `trajectories`；每条 trajectory 的 `time_offsets[i]` 与 `poses[i]` 成对，偏移相对于 array 的 `header.stamp`，坐标均在 `header.frame_id` 中。先自己执行：

```bash
ros2 interface show predictive_nav_msgs/msg/PredictedTrajectoryArray
ros2 interface show predictive_nav_msgs/msg/PredictedTrajectory
```

然后将 `is_prediction_usable()` 拆成小判断，避免一个 100 行 `if`：

```text
is_header_usable(message.header, now, reason)
is_trajectory_shape_usable(trajectory, reason)
is_finite_trajectory(trajectory, reason)
```

你可以先只实现公共 header 和数组长度检查，编译通过后再逐项补 `finite` 检查。每补一项就写一个可读的 reason，例如 `wrong_frame`、`stale`、`shape_mismatch`，不要只打印“invalid”。

## 时间判断实现提示

```text
age = now - header.stamp
age > max_prediction_age_s     → stale，拒绝
age < -allowed_future_skew_s   → future_stamp，拒绝
否则                           → 通过时间检查
```

`use_sim_time:=true` 时必须用 ROS 时钟，而不是 `std::chrono` 的墙钟。否则 Gazebo 暂停、rosbag 回放或仿真时间重置时，age 会错误。

## 本节的最小故障测试

先不人为改坏消息定义。最简单的故障注入是停掉 `prediction_node`，等待超过 `max_prediction_age_s`，确认只出现节流 warning，MPPI 仍保留静态避障能力。把 warning 文本和你的降级选择写进笔记。
