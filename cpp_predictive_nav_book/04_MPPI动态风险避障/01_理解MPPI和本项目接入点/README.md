# 01：理解 MPPI 和本项目接入点

先用项目现有导航链路解释 MPPI 的位置：全局规划给方向，MPPI 在局部不断挑选短时间驾驶轨迹。

**critic（评价器）**是 MPPI 的一个可插拔评分规则。本项目只新增动态风险这一条规则，不重写 Nav2 控制器。

完成标准：能画出 `/predictions → critic → MPPI → /cmd_vel`，并知道 critic 不应直接控制小车。

## 本步是否写代码

不写 C++。这是后续所有代码的设计前提；如果此处职责混乱，后面即使插件能编译，也会把控制链路接错。

## 你要亲自完成的内容

在自己的笔记中手画下面两张图，不要照抄后面的答案：

1. 从 RViz 发送目标到 Gazebo 底盘执行的完整链路；
2. `prediction_node`、`DynamicRiskCritic`、MPPI、`/cmd_vel` 的输入输出图。

然后回答：

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
