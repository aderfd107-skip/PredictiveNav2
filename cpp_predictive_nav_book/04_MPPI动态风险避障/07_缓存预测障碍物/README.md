# 07：缓存预测障碍物

> 本步目标：让已加载的 Critic 保存 `/dynamic_obstacles/predictions` 的最新一帧；先只观察数据，不计算风险。

## 为什么不能在 `score()` 中等待 topic

MPPI 每个控制周期需要快速为大量候选轨迹评分。ROS topic 回调却是异步到达的：它可能慢、
可能暂时没有新消息，也可能正好与评分同时发生。

因此采用“回调写入、控制循环读快照”的结构：

```text
prediction callback
  ↓ mutex 保护的快速赋值
latest_prediction_
  ↓ 复制 shared pointer 后立刻释放锁
score() 在锁外读取快照并评分
```

`score()` 不应等待下一帧 prediction，也不能在持锁状态下执行后面的三层距离循环。

## 将新增哪些成员

```cpp
using PredictionArray = predictive_nav_msgs::msg::PredictedTrajectoryArray;

rclcpp::Subscription<PredictionArray>::SharedPtr prediction_subscription_;
mutable std::mutex prediction_mutex_;
PredictionArray::ConstSharedPtr latest_prediction_;
```

`ConstSharedPtr` 表示 Critic 不修改 topic 原始消息。复制 shared pointer 很快；复制完整预测
数组在控制循环中可能很贵。

## 回调与快照各做什么

```text
prediction_callback(message)
  空消息 → 返回
  加锁 → latest_prediction_ = message → 解锁
  锁外节流打印 stamp、frame、轨迹数量

latest_prediction_snapshot()
  加锁 → 复制 latest_prediction_ 指针 → 解锁 → 返回
```

第 08 步才检查 frame、时效和数值有效性。本步收到空数组也应正常保存和记录，因为“收到
一帧空 prediction”与“从未收到消息”是两种不同状态。

## 如何验证订阅

prediction 节点运行后：

```bash
ros2 topic info -v /dynamic_obstacles/predictions
ros2 topic echo --once /dynamic_obstacles/predictions
```

应看到 publisher 为 `prediction_node`，subscriber 包含 controller_server 中的 Critic。
停掉 prediction_node 后，Critic 不应崩溃，也不应继续声称收到递增的 stamp。

## 本步完成标准

- [ ] Critic 可订阅 `/dynamic_obstacles/predictions`；
- [ ] 回调和 `score()` 不会并发读写同一消息对象；
- [ ] mutex 只保护指针操作；
- [ ] 空 Critic 仍不修改 `data.costs`；
- [ ] 已能区分无消息、空消息和最新消息三种状态。

## 下一步

[08_检查预测时效与坐标系](../08_检查预测时效与坐标系/README.md)：不是每一帧缓存都能
参与控制，下一步要拒绝旧数据、错误 frame 和异常数值。
