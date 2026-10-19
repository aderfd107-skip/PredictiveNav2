# 07：缓存预测障碍物

> **本步目标：** 让 critic 订阅 prediction topic，并安全保存最新一帧；此时只缓存和观察，不计分。
>
> **AI 会做什么：** 生成订阅回调、互斥锁、快照函数与节流日志，并说明锁为什么不能包住评分循环。
>
> **你只需要做到：** 通过 topic echo、日志或断开预测节点，验证 critic 收到新数据、不会读到损坏数据，并能说明无预测时采取什么行为。

critic 订阅 `/dynamic_obstacles/predictions`，只保存最新一帧有效预测，评分时读取这份缓存。

**缓存**是暂存最新数据，避免每次评分等待 topic 回调。要使用互斥保护或等价设计，防止回调和评分同时读写造成错误。

完成标准：不接入风险公式时，日志能确认 critic 收到最新预测及其时间戳。

## 本步 AI 生成与审读

这是并发与 ROS 2 数据流的核心逻辑。AI 实现“回调写入最新 prediction，控制循环快速取一份快照”；你要确认 `score()` 不等待 topic，且 mutex 只保护快速的指针读取/写入。

## 需要的头文件

```cpp
#include <memory>
#include <mutex>

#include "predictive_nav_msgs/msg/predicted_trajectory_array.hpp"
#include "rclcpp/rclcpp.hpp"
```

## AI 将生成的成员与函数

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

## 你必须读懂的伪代码

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

生成后先审读 callback 和 snapshot 函数，不要先加入第 08 步的有效性判断。

## 生成与验证顺序：先让数据进来，再谈风险

让 AI 按下列顺序生成，每完成一项就编译或运行一次：

1. 在头文件加入类型别名、subscription、`std::mutex` 和最新消息指针；
2. 在 `.cpp` 的 `initialize()` 中先取得 `parent_.lock()`，若为空则记录错误并返回；
3. 用这个 lifecycle node 创建 `/dynamic_obstacles/predictions` 订阅；
4. 只实现 `prediction_callback()`：锁内只赋值 shared pointer；
5. 实现 `latest_prediction_snapshot()`：锁内复制指针，解锁后返回；
6. 在 `score()` 里调用 snapshot，但暂时不校验、不计分。

订阅 QoS 需要与 prediction publisher 相容。先从现有话题实际情况确认，而不是猜：

```bash
ros2 topic info -v /dynamic_obstacles/predictions
```

## AI 生成代码时必须保持的形状

下面给出 AI 生成函数体时必须满足的边界。重点是 `lock_guard` 的作用域必须小。

```cpp
void DynamicRiskCritic::prediction_callback(PredictionArray::ConstSharedPtr message)
{
  // TODO: 空指针处理
  // TODO: 一个仅包住 latest_prediction_ 赋值的 lock_guard
  // TODO: 锁外节流日志
}

PredictionArray::ConstSharedPtr
DynamicRiskCritic::latest_prediction_snapshot() const
{
  // TODO: 加锁、复制 shared pointer、返回
}
```

不要在 callback 中遍历所有 trajectory；也不要在 `score()` 中保留 mutex 后做两层或三层循环。这个习惯对真机 20 Hz 控制循环很重要。

## 本节验收记录

运行时分别截图：prediction node 正常、prediction node 停止后 2 秒、空 array 发布时。你必须能区分“没有收到过任何消息”“收到空 prediction 帧”“最新帧已经过期”；第 08 节才将这些状态转成可用/不可用判断。
