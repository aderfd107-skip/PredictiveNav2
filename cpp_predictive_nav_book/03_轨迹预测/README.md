# 第三模块：从轨迹状态到未来预测

## 这块要解决什么

第二模块的 `track` 说的是“障碍物现在在哪里、速度大约是多少”。本模块要回答“如果它暂时保持这种运动，未来 0.2、0.4、0.6 秒可能在哪里，以及这个判断有多不确定”。

这里的**轨迹预测**不是控制小车，也不是保证看穿未来；它只是把 track 的状态按短时间模型向前推算，并发布给下一模块的风险评估使用。

## 完成后的数据流

```text
/dynamic_obstacles/tracks
        ↓
predictive_nav_prediction
        ├── /dynamic_obstacles/predictions         正式预测接口
        └── /dynamic_obstacles/prediction_markers  RViz 展示接口
        ↓
第 04 模块的 DynamicRiskCritic
```

## 本章已经做了什么

你已经实现了一个独立的 ROS 2 `prediction_node`。它不依赖 Gazebo 真值，而是只读取第二章 tracker 给出的 Track 状态，并为每个合格 Track 预测未来 `2.0 s` 的运动。

| 部分 | 已完成内容 | 为什么重要 |
|---|---|---|
| 输入接口 | 订阅 `/dynamic_obstacles/tracks`，检查 `frame_id=odom`、消息时间戳和 Track 状态 | 预测必须建立在统一坐标系、可信时刻的数据上 |
| 时间轴 | 生成 `0.2, 0.4, ..., 2.0 s` 共 10 个预测时刻 | 后续风险评估能比较“同一个未来时刻”的风险 |
| 运动模型 | 使用 CV（Constant Velocity，匀速）模型：`p(t)=p(0)+v·t` | 是短时动态障碍物预测的清晰 baseline |
| 不确定性 | 从 Track 的 `[px, py, vx, vy]` 四维协方差，传播出每个未来位置的二维协方差 | 不只输出“可能在哪”，还输出“这个判断有多可靠” |
| 质量门限 | 丢弃过期、时间倒退、frame 错误、NaN/Inf、年龄不足、miss 过多、置信度低或速度异常的 Track | 防止坏输入被包装成貌似正常的预测 |
| 正式输出 | 发布 `/dynamic_obstacles/predictions`，每条轨迹含 Track ID、时间偏移、未来 Pose 和协方差 | 为第 04 模块提供稳定、可被程序订阅的接口 |
| RViz 展示 | 发布独立的 MarkerArray：筛选少量动态候选、青色轨迹点线、紫色 2σ 协方差椭圆 | 让人能看懂结果，同时不污染正式算法接口 |
| 可复现排错 | 制定 `/clock + /tracks` 的 rosbag 最小回放流程 | 后续改参数时可用同一段输入做公平比较 |

## 你通过这一章学会了什么

### 1. 轨迹预测的基本建模方式

- `CV` 不是“机器学习模型”，而是常速度运动假设；它适合作为可解释、低延迟的短期预测 baseline。
- 当前位置和速度只能给出预测中心线；协方差随预测时间增长，才表达了目标转弯、加减速等未建模因素带来的风险。
- 预测值与真值不同是正常的。正确问题不是“它为什么不能永远预测准”，而是“在多远的时间范围内、带多大不确定性时仍然有工程价值”。

### 2. 协方差从跟踪传到预测

你现在可以区分两种协方差：

- tracker 内部使用的是四维状态协方差 `P=[px, py, vx, vy]`；
- prediction 输出给下游的是每个未来位置的二维 `P_position=[x, y]` 协方差，并放入 ROS 的 `PoseWithCovariance` 对应位置。

你还实现了 CV 传播的核心思想：已有的位置/速度不确定性会被时间放大，另外未知加速度过程噪声会继续让远期位置方差变大。

### 3. ROS 2 工程接口与数据质量

- 设计并使用自定义消息 `PredictedTrajectory`、`PredictedTrajectoryArray`；
- 区分**正式消息接口**与**纯 RViz 调试接口**，Marker 不参与算法决策；
- 使用 `header.stamp` 和 ROS 仿真时间，而不是拿终端 wall-clock 计算数据年龄；
- 用空的 trajectory array 表达“节点正常收到一帧，但没有安全可预测的目标”，避免把“空结果”和“节点死掉”混为一谈；
- 用 rosbag 固定输入，并避免回放旧输出与新输出混在同一个 topic 上。

### 4. C++ / ROS 2 代码能力

- 使用 `std::vector` 组织预测时间轴和未来点；
- 用 range-based for 遍历时间轴与轨迹；
- 用 lambda 配合 `std::max_element`、`std::sort` 做候选选择；
- 用 `std::isfinite`、`std::hypot`、`std::clamp` 处理数值安全与几何量；
- 使用 ROS 2 publisher、subscriber、timer、parameter、QoS 和 throttle log；
- 通过 `colcon build --packages-select ...` 做增量编译，并在改动后重启对应节点验证。

## 可以怎样向面试官介绍这一章

> 我在 ROS 2 中实现了一个独立的动态障碍物短时预测模块。它订阅多目标跟踪输出，在 `odom` 坐标系下用常速度模型生成 2 秒、0.2 秒间隔的未来轨迹；同时把跟踪器的四维状态协方差和加速度过程噪声传播为每个未来位置的不确定性。工程上加入了时间戳、坐标系、NaN、置信度、轨迹年龄和丢失帧等质量门限，并把正式预测消息与 RViz 可视化分离。最后通过 rosbag 固定 tracks 输入，支持离线复现和参数对比。

这段话的前提是你能解释 CV 假设为何有局限、为什么协方差要传播，以及为什么展示 Marker 不能当作算法输入；这些内容你都已经在本章代码中做过。

## 当前完成边界与下一步

本章完成的是“**感知/跟踪结果 → 未来轨迹与不确定性**”。它还没有让小车因为预测结果而减速或绕行：

- prediction_node 不发布 `/cmd_vel`；
- RViz 的椭圆只给人看，不会进入 costmap；
- CV 对突然转弯、突然停止和长期运动的效果有限；
- 当前展示视图只显示少量动态候选，正式消息仍保留所有通过质量门限的轨迹。

第 04 模块将把 `/dynamic_obstacles/predictions` 接入风险评价和局部规划/MPPI，届时预测才会真实影响机器人避障行为。

## 本章验收清单

- [x] prediction_node 能启动并订阅 tracks；
- [x] 能发布 10 个未来时刻的预测轨迹与位置协方差；
- [x] 能拒绝过期、异常或低质量 Track；
- [x] RViz 能显示筛选后的预测轨迹和协方差椭圆；
- [ ] 录制第一份 `step11_prediction_case_01`，并完成一次 `/clock + /tracks` 最小回放；
- [ ] 保存一张干净的展示截图和一张包含调试层的排错截图。

## 步骤路线

1. [00_这一模块要做什么](00_这一模块要做什么/README.md)
2. [01_观察tracks输入](01_观察tracks输入/README.md)
3. [02_创建prediction包骨架](02_创建prediction包骨架/README.md)
4. [03_定义预测消息](03_定义预测消息/README.md)
5. [04_订阅track并打印](04_订阅track并打印/README.md)
6. [05_确定预测时间轴](05_确定预测时间轴/README.md)
7. [06_CV传播未来位置](06_CV传播未来位置/README.md)
8. [07_传播位置不确定性](07_传播位置不确定性/README.md)
9. [08_处理过期与异常预测](08_处理过期与异常预测/README.md)
10. [09_发布预测结果](09_发布预测结果/README.md)
11. [10_RViz显示预测轨迹](10_RViz显示预测轨迹/README.md)
12. [11_rosbag回放与排错](11_rosbag回放与排错/README.md)

## 先记住的边界

- 预测只使用第 02 模块发布的 track，不读取 Gazebo 真值。
- CV 是短时间近似；方块转向、被遮挡时，预测自然会变差。
- 本模块只提供“风险信息”，不直接发布 `/cmd_vel`。
