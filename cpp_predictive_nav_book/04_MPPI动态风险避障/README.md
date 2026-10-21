# 第四模块：让 Nav2 评估动态风险

## 这块要解决什么

第三模块已经能从 `/dynamic_obstacles/tracks` 产生带时间轴和协方差的
`/dynamic_obstacles/predictions`。但预测目前只显示在 RViz，Nav2 不会因为它而
改变机器人动作。

本章要新增一个 Nav2 MPPI Critic：它在 MPPI 给许多候选机器人轨迹评分时，额外判断
“机器人和障碍物会不会在**同一未来时刻**靠得太近”。风险高的候选轨迹得到更高 cost，
MPPI 自然更倾向减速、等待或选择另一条局部动作。

```text
LiDAR → cluster → tracking → prediction
                              ↓
                    DynamicRiskCritic
                              ↓  给 MPPI 候选轨迹加 risk cost
global path → Nav2 MPPI → /cmd_vel → 底盘 / Gazebo
```

## 完成后的数据流

```text
/dynamic_obstacles/predictions
          ↓ 最新一帧缓存、时效和 frame 检查
DynamicRiskCritic::score(CriticData)
          ↓ 距离、时间对齐、TTC、协方差风险
data.costs(i) += dynamic_risk_cost
          ↓
Nav2 MPPI 在原有静态障碍、路径跟随和速度约束之外作出选择
```

这里的 Critic **不直接发布 `/cmd_vel`**。它只改变候选动作评分；真正的速度命令仍由
Nav2 MPPI 输出，静态 costmap、运动学约束与已有 `scan_safety_guard` 仍保留。

## 本章已经做了什么

目前仓库已经有：

1. `nav2_mppi_baseline.yaml`：差速底盘的原版 MPPI 基线；
2. `predictive_nav_mppi`：可构建的共享库包；
3. `DynamicRiskCritic` 空类：继承 Nav2 Critic 接口，但不修改任何 cost。

因此目前能验证“MPPI 可运行”和“自定义 Critic 可编译”，还不能声称机器人已经根据
预测做动态避障。第 06 步才让 Nav2 加载该类，第 07–11 步才逐步添加预测与风险逻辑。

## 你通过这一章会学到什么

### 1. 全局规划、局部规划与底盘控制的边界

全局规划给总体方向；MPPI 选择短时动作；底盘执行 `/cmd_vel`。动态风险 Critic 位于
MPPI 内部，只提供评分。

### 2. MPPI 为什么适合接入预测

MPPI 会前向模拟未来短轨迹，prediction 也给出未来障碍物位置。两者以相同时间点
对齐后，能区分“位置相同但经过时间不同”与“未来同一时刻相遇”。

### 3. ROS 2 / Nav2 插件工程

你会见到 C++ 共享库、继承、虚函数、pluginlib XML、CMake 安装规则，以及运行时加载
失败时如何定位问题。

### 4. 诚实的动态避障实验

最终在相同地图、起终点、actor 路线和速度下，对比原版 MPPI 与动态风险版的最小距离、
到达率、耗时、停顿和控制周期耗时。

## 可以怎样向面试官介绍这一章

> 我将 LiDAR 跟踪和短时预测结果接入 Nav2 MPPI 的 Critic 评分接口，而不是绕开 Nav2
> 自己发布速度。对于每条候选机器人轨迹，系统按时间对齐比较其与动态障碍物预测轨迹
> 的距离，并把 TTC 和预测协方差转成额外风险代价。预测无效、过期或 frame 不一致时，
> Critic 会拒绝该数据并保留原有静态安全层；最后使用原版 MPPI 作为 baseline 量化收益。

## 当前完成边界与下一步

当前处于“空 Critic 已编译、尚未由 Nav2 加载”的阶段。下一步是第 06 节：补齐
pluginlib 导出和 YAML 配置，先加载空 Critic，再开始订阅 prediction。

## 本章验收清单

- [ ] 原版 MPPI 可完成导航；
- [ ] `DynamicRiskCritic` 被 controller_server 成功加载；
- [ ] Critic 只读取新鲜、坐标系正确、数值有限的 prediction；
- [ ] 同一时刻接近的候选轨迹获得更高动态风险 cost；
- [ ] 动态场景中出现可解释的减速、等待或绕行；
- [ ] 与原版 MPPI 的对照包含收益、失败样本和代价。

## 步骤路线

`00–03` 建立并观察原版 MPPI；`04–06` 创建、定义和加载空 Critic；
`07–11` 接收 prediction 并计算风险；`12–14` 配置、运行和观察行为；
`15–16` 排错并完成对照实验。

## 先记住的边界

- DynamicRiskCritic 是研究型连续风险评分，不是认证急停系统；
- 预测不是 Gazebo 真值，预测越不确定越应保守；
- Critic 可以累加自己的 cost，不能覆盖其他 Critic 的 cost；
- “车不走”时，先在原版 MPPI baseline 中复现或排除问题。
