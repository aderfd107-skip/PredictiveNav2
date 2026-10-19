# 01：理解 MPPI 和本项目接入点

> **本步目标：** 理解全局规划、MPPI、`DynamicRiskCritic` 与底盘控制的位置关系；本步不修改代码。
>
> **AI 会做什么：** 根据本仓库画出架构图，并把每个 ROS topic、输入和输出标出来。
>
> **你只需要做到：** 能回答“为什么 critic 只评分、不直接发布 `/cmd_vel`”，以及“动态预测为何不能直接替代局部控制器”。

先用项目现有导航链路解释 MPPI 的位置：全局规划给方向，MPPI 在局部不断挑选短时间驾驶轨迹。

**critic（评价器）**是 MPPI 的一个可插拔评分规则。本项目只新增动态风险这一条规则，不重写 Nav2 控制器。

完成标准：能画出 `/predictions → critic → MPPI → /cmd_vel`，并知道 critic 不应直接控制小车。

## 本步是否写代码

不写 C++。这是后续所有代码的设计前提；如果此处职责混乱，后面即使插件能编译，也会把控制链路接错。

## AI 生成的架构图，你要读懂什么

让 AI 根据下面的接口生成两张架构图：

1. 从 RViz 发送目标到 Gazebo 底盘执行的完整链路；
2. `prediction_node`、`DynamicRiskCritic`、MPPI、`/cmd_vel` 的输入输出图。

读图后，你应能回答：

- 为什么 Critic 不能自己再创建一个 `/cmd_vel` publisher？
- 若动态预测话题停了，谁应该决定局部速度？
- `map`、`odom`、`base_footprint` 分别在这条链中承担什么角色？

## 本步必须说清的接口

```text
/dynamic_obstacles/predictions  →  DynamicRiskCritic 的订阅缓存
Nav2 global path                →  MPPI 的路径相关 Critic
MPPI candidate trajectories     →  DynamicRiskCritic::score(...)
MPPI                             →  /cmd_vel
```

`DynamicRiskCritic` 只向 MPPI 的候选 cost 数组加非负分数；它不规划全局路径、不修改 prediction、不直接控制电机。

## 自检

能在不看 README 的情况下用 60 秒解释：全局规划、局部 MPPI、Critic 与底盘控制四者的边界，就可以进入第 02 步。

## 用文字核对架构图

让 AI 按下面模板补全箭头两侧的“输入 / 输出”。这张图以后会成为你面试时解释项目的底稿。

```text
RViz 目标点
  → BT Navigator
  → planner_server（Navfn / A*）
  → __________________________
  → controller_server（MPPI）
  → __________________________
  → scan_safety_guard
  → Gazebo DiffDrive / 真机底盘
```

再单独画出动态数据支路：

```text
prediction_node --发布--> /dynamic_obstacles/predictions
                                      ↓
                         DynamicRiskCritic 的 __________
                                      ↓
                  MPPI 的一批候选轨迹 + 每条轨迹的 ______
                                      ↓
                               MPPI 选择 /cmd_vel
```

读完后，用自己的话分别回答：全局规划为何不发布轮速；Critic 为何只改 cost；底盘为什么不该知道 prediction topic。答案不要求术语完美，但三层输入输出不能混淆。

## 本节完成标准

AI 生成两张图后，你能用不超过 120 字解释 DWB 与 MPPI 都位于哪一层即可。此节没有任何 C++ 文件要创建。
