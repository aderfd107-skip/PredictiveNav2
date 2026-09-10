# 07：缓存预测障碍物

critic 订阅 `/dynamic_obstacles/predictions`，只保存最新一帧有效预测，评分时读取这份缓存。

**缓存**是暂存最新数据，避免每次评分等待 topic 回调。要使用互斥保护或等价设计，防止回调和评分同时读写造成错误。

完成标准：不接入风险公式时，日志能确认 critic 收到最新预测及其时间戳。

## 本步必须手写

这是并发与 ROS 2 数据流的核心练习。你要自己完成“回调写入最新 prediction，控制循环快速取一份快照”的代码；不要在 `score()` 中等待 topic，也不要长时间持有锁。

## 需要的头文件

```cpp
#include <memory>
#include <mutex>

#include "predictive_nav_msgs/msg/predicted_trajectory_array.hpp"
#include "rclcpp/rclcpp.hpp"
```

## 建议你自己声明的成员与函数

```cpp
using PredictionArray = predictive_nav_msgs::msg::PredictedTrajectoryArray;

void prediction_callback(PredictionArray::ConstSharedPtr message);
PredictionArray::ConstSharedPtr latest_prediction_snapshot() const;

rclcpp::Subscription<PredictionArray>::SharedPtr prediction_subscription_;
mutable std::mutex prediction_mutex_;
PredictionArray::ConstSharedPtr latest_prediction_;
```

在 `initialize()` 中，通过 `parent_.lock()` 得到 lifecycle node，再创建订阅者。topic 固定为：

```text
/dynamic_obstacles/predictions
```

## 你必须写出的伪代码

```text
callback(message):
  若 message 空指针，直接返回
  lock
  latest_prediction_ = message
  unlock
  节流打印 stamp、frame、trajectory 数量

score(...):
  snapshot = latest_prediction_snapshot()
  锁已经释放
  后续只读取 snapshot，绝不在锁内做三层风险循环
```

`ConstSharedPtr` 的意思是评分函数不会修改来自 topic 的原始消息；复制 shared pointer 很快，复制整帧数组可能很贵。

## 必须检查的运行现象

- prediction_node 正常运行时，Critic 日志显示递增 stamp；
- 停掉 prediction_node 后，Critic 不崩溃，也不继续声称收到新帧；
- 空 Critic 仍不修改 `data.costs`，机器人行为应与 baseline 一致。

写完先发你的 callback 和 snapshot 函数，不要先写第 08 步的有效性判断。
