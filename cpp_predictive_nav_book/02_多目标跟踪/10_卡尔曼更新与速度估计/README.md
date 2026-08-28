# 10：卡尔曼更新与速度估计

## 先用一句人话说明

第 08 步说：“按旧速度猜，方块现在应该在这里。”第 09 步说：“这个 LiDAR cluster 很可能就是那个方块。”现在要做的是：**让预测和新测量各退一步，合成一个更可靠的新状态。**

程序不会把预测位置直接替换成 LiDAR 中心，因为聚类中心也会抖；也不会无视新测量继续按旧速度走。这个“按各自可信程度折中”的过程叫 **Kalman update（卡尔曼更新）**。

## 四个新名词

| 名词 | 白话含义 |
|---|---|
| 预测 | 根据上一帧位置、速度和 `dt` 推测当前位置 |
| 测量 | 当前匹配 LiDAR cluster 的中心点 |
| 创新量（innovation） | 测量减预测，即“这次看到的和原来猜的差多少” |
| 卡尔曼增益（`K`） | 决定这次向测量靠近多少的系数 |

例如预测 y 是 `-2.70 m`、测量 y 是 `-2.76 m`，创新量为 `-0.06 m`。更新后的位置会向 `-2.76` 靠近，但通常不会一次完全跳过去。

## 为什么只测位置，也能修正速度

LiDAR 只给 `(x, y)`，不直接给 `(vx, vy)`。但如果连续几帧真实位置总在预测前方，程序会推断旧速度偏小；反之则偏大。第 08 步的协方差已经建立位置和速度的关系，所以位置测量可以间接修正速度。

这比第 06 步直接拿两帧相减稳定得多：单帧激光抖动不会立即变成巨大速度。

## 核心公式：知道用途即可

```text
innovation = measurement - H × predicted_state
K = P × Hᵀ × (H × P × Hᵀ + R)⁻¹
new_state = predicted_state + K × innovation
```

- `H`：LiDAR 只能看到 `px`、`py`，看不到速度；
- `P`：预测后的状态不确定性；
- `R`：对 LiDAR cluster 中心误差的假设；
- `K`：综合 `P`、`R` 后得到的“相信测量程度”。

代码用 Joseph form 更新协方差。这是较稳定的工程写法；现在只需理解更新后 `P` 仍要表示合理的不确定性。

## 本步代码做了什么

文件：[tracking_node.cpp](../../../src/predictive_nav_tracking/src/tracking_node.cpp)

`update_debug_cv_state()` 只在第 09 步得到 `matched` 后执行：

1. 取匹配 cluster 的中心作为二维测量；
2. 算测量和预测的创新量；
3. 算 Kalman gain；
4. 同时修正位置、速度和协方差；
5. 若矩阵求解或结果出现 NaN/Inf，拒绝这次更新并报告日志。

它仍是教学状态 `debug_cv_state_`，ID 为 0，不在真正的 `tracks_` 数组中。第 11 步才会对多个真实 Track 组装相同循环。

## 新参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `measurement_position_stddev_m` | `0.15` | 认为 LiDAR cluster 中心的位置标准差（米） |

程序内部会使用方差 `0.15² = 0.0225 m²`。参数更小表示更相信测量；参数更大表示更相信预测、结果更平滑但纠错更慢。它要靠 rosbag 和真机调参，不能认为真实误差永远恰好是 0.15 m。

## 构建与运行

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

随后按第 06 步启动动态仿真、感知节点和 tracking 节点。关联成功时，每十帧应出现：

```text
Kalman update (debug only) |
measurement=(2.27, -2.80) m |
innovation=(0.02, -0.03) m |
before=(2.25, -2.77, 0.01, 0.30) |
after=(2.26, -2.79, 0.02, 0.27) |
K_velocity=(...)
```

- `measurement`：本帧匹配 cluster 的中心；
- `innovation`：测量与预测的差；
- `before`、`after`：固定顺序为 `(px, py, vx, vy)`；
- `after` 的位置会向 measurement 靠近，速度也可能变化；
- `K_velocity` 表示位置创新量影响速度的程度，不要求现在手算。

若看到 `skipped because this frame has no gated association`，表示本帧没有可信 cluster，程序选择不更新以避免把错误目标塞进状态。这是保护行为。

## 一个可选实验

```bash
ros2 run predictive_nav_tracking tracking_node --ros-args \
  -p measurement_position_stddev_m:=0.40
```

这会让程序更不相信 LiDAR；观察相同量级 innovation 下，`after` 相比 `before` 的移动通常更小。结束后不带参数重启即可恢复默认值。

## 本步还没有做什么

- 未创建/删除真实多目标 Track：第 11 步实现；
- 未发布 `/dynamic_obstacles/tracks`：第 12 步实现；
- 仍用欧氏距离 + gate 关联，未做 Mahalanobis 或匈牙利算法；
- 没有可信关联时跳过 update，这是正确行为。

## 本章必须懂的 ROS 2 / Nav2 知识

- 预测、测量和更新必须属于同一时间语义、同一个 `odom` 坐标系；
- 状态更新不是“收到消息就覆盖变量”，必须检查时间、关联和数值有效性；
- 此处仍只影响 tracking 节点内部，不改变 Nav2 或 `/cmd_vel`。

## 完成检查

- [ ] 构建成功并看到 `Kalman update` 日志；
- [ ] 能解释 measurement、innovation、before、after；
- [ ] 知道位置测量会间接修正速度；
- [ ] 知道没有匹配就跳过 update。
