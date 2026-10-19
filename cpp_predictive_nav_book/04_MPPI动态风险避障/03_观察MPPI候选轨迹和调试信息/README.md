# 03：观察 MPPI 候选轨迹和调试信息

> **本步目标：** 打开原版 MPPI 的调试可视化，分清全局路径、机器人候选轨迹、最优局部动作和障碍物预测；本步不写 C++。
>
> **AI 会做什么：** 修改 YAML 调试开关、给出 RViz 显示配置与检查命令，并根据你的截图整理实验记录。
>
> **你只需要做到：** 能拿一次实际运行截图，说出机器人为什么加速、减速、转向或停住，而不是只说“MPPI 算出来了”。

打开 MPPI 支持的调试输出或 RViz 显示，先观察它如何从大量候选轨迹中选择控制动作。

**候选轨迹**是控制器内部假设“接下来这样开”的短路径，不等同于全局路径，也不等同于障碍物预测轨迹。

完成标准：能区分三者，并知道到哪里查看 MPPI 是否正常工作。

## 本步是否写代码

不写 C++。让 AI 修改 MPPI baseline YAML 的调试开关、给出 RViz/终端检查步骤；你运行并观察结果，再让 AI 恢复不会长期拖慢控制循环的设置。

## AI 生成、你审读的配置任务

在 `FollowPath` 的 MPPI 参数中查阅并试用 Jazzy 支持的轨迹可视化配置：

```yaml
visualize: true
TrajectoryVisualizer:
  trajectory_step: 5
  time_step: 3
```

`visualize` 与 `TrajectoryVisualizer` 都属于 `FollowPath`；后者缩减发布的候选轨迹数量和每条轨迹显示的时间点。

> **Jazzy 边界：** 本机原版 `nav2_mppi_controller` 没有 `publish_critics_stats` 参数。不要添加它，也不要把它放进 `TrajectoryVisualizer`：它会成为未使用/无效的 YAML 项，而不会产生逐 critic 统计。第 09 节实现自定义 critic 时，才由项目自己发布需要的调试统计。

不要把轨迹可视化默认永久打开：它为了显示候选轨迹需要额外回推数据，会增加控制循环负担。记录打开与关闭时的差别。

## 必须区分的三条轨迹

| 名称 | 产生者 | 含义 |
|---|---|---|
| 全局路径 | Nav2 planner | 从当前位置到目标的大方向 |
| MPPI 候选轨迹 | MPPI 内部采样 | 许多“接下来短时间这样开”的机器人假设 |
| 障碍物预测轨迹 | 第 03 章 prediction_node | 障碍物未来可能的位置，不是机器人轨迹 |

## AI 应写进实验笔记、你要能回答的诊断问题

- 机器人不动时，MPPI 是没有候选、候选都碰撞、还是 action 已终止？
- `/cmd_vel` 非零而 Gazebo 不动时，应该检查控制链还是 Critic？
- 一条候选轨迹的时间离散步长从哪里来？它和 prediction 的 `time_offsets` 为什么不能混为一谈？

没有新 `#include` 或函数。本步的目标是让你在第 09 步看到 `CriticData` 时知道它装的是哪一种轨迹。

## 如何运行与保存证据

1. 只在 `nav2_mppi_baseline.yaml` 中打开一个调试开关，不要同时改速度、critic、地图；
2. 重启 MPPI launch，发送一个短距离目标；
3. 新终端执行下面命令，先找出你本机实际发布的话题名称；

```bash
source /opt/ros/jazzy/setup.bash
source ~/PredictiveNav2/install/setup.bash
ros2 topic list | rg -i 'mppi|trajectory|critic|cmd_vel'
ros2 topic hz /cmd_vel
```

4. 在 RViz 中把全局 Path、Local Path、MPPI 调试轨迹分别改成不同颜色；
5. 截一张“机器人行驶中”的图，圈出三者。最后关掉高频可视化开关再测一次 `/cmd_vel` 频率。

让 AI 根据你的终端输出和截图生成下面这个最小表格；保存前你要核对：

| 观察项 | 实际话题 / 显示名 | 看见了什么 | 结论 |
|---|---|---|---|
| 全局路径 |  |  |  |
| 局部控制输出 | `/cmd_vel` |  |  |
| MPPI 调试候选 |  |  |  |
| 动态预测 | `/dynamic_obstacles/predictions` |  | 与 MPPI 候选不是同一类轨迹 |

此节不追求把每一个 MPPI 内部 topic 都显示出来；重点是你知道“没有看到某个可视化话题”不等于控制器没有运行。
