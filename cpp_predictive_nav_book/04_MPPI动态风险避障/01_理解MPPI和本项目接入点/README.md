# 01：理解 MPPI 和本项目接入点

> 本步目标：区分全局规划、局部 MPPI、DynamicRiskCritic 与底盘控制的职责；本步不修改代码。

## 一条目标指令经过哪些模块

```text
RViz Goal
  ↓
BT Navigator：管理 NavigateToPose 任务和 recovery
  ↓
planner_server：在全局地图上生成 Global Path
  ↓
controller_server / MPPI：不断选择下一小段速度
  ↓
/cmd_vel → scan_safety_guard → Gazebo DiffDrive / 真机底盘
```

全局路径只说明总体应走哪边，不会精确规定每个控制周期的线速度和角速度。遇到障碍、
转角或定位误差时，局部控制器必须重新决定眼前动作。

## DWB 与 MPPI 都在哪里

DWB 和 MPPI 都是 Nav2 的局部控制器。DWB 采样速度命令并用 Critic 打分；MPPI 采样短时
控制序列，前向模拟出候选轨迹后评分。

```text
global path
    ↓
controller_server
    ├── DWBController       旧基线
    └── MPPIController      当前第 04 章基线
```

它们都不是全局规划器，也不是底盘驱动器。把 DWB 换成 MPPI 后，机器人仍是差速底盘；
变化的是选择下一条局部速度的方法。

## Critic 位于 MPPI 的哪里

```text
候选轨迹 i
  ├── ConstraintCritic：速度、加速度是否可行
  ├── CostCritic：是否靠近静态 costmap 障碍
  ├── Path / Goal Critic：是否向路径和目标推进
  └── DynamicRiskCritic：未来是否会靠近预测障碍物
```

每个 Critic 都只对 `data.costs(i)` 增加分数。MPPI 再将总分较好的候选变成当前 `/cmd_vel`。

## 三个坐标系为什么会出现

| 坐标系 | 用途 |
|---|---|
| `map` | AMCL 定位与全局路径，长期固定的地图坐标 |
| `odom` | 局部连续运动坐标，tracking/prediction 使用它 |
| `base_footprint` | 机器人车体中心坐标 |

prediction 与 MPPI 局部轨迹要在同一坐标系下才可相减。后续以 `odom` 作为预测比较坐标系；
不能把 `map` 数字直接与 `odom` 数字相减。

## 本步完成标准

- 能说明 global path 与 `/cmd_vel` 之间还有 controller_server；
- 能说明 DWB、MPPI 都是局部规划/控制方法；
- 能说明 Critic 评分但不直接发布速度；
- 能说明为什么预测坐标系必须与候选轨迹坐标系一致。

## 下一步

[02_建立原版MPPI基线](../02_建立原版MPPI基线/README.md)：先运行没有动态风险的原版 MPPI，
保存之后比较必须使用的对照组。
