# 03：观察 MPPI 候选轨迹和调试信息

> 本步目标：在加入自定义 Critic 前，先看清 MPPI 正在比较什么；本步不新增 C++ 代码。

## 先区分四种画面

RViz 中最容易混淆的是下面四种对象：

| 名称 | 来自哪里 | 表示什么 |
|---|---|---|
| Global Path | planner_server | 到目标的大方向 |
| Local Path | controller_server | 当前局部跟随参考 |
| MPPI 候选轨迹 | MPPI 调试输出 | 很多“如果这样开”的机器人短轨迹 |
| 障碍物预测轨迹 | prediction_node | 障碍物未来可能的位置 |

候选轨迹不是全局路径，也不是机器人最终实际走过的轨迹。MPPI 每个控制周期都会重新采样，
画面中的短线只是它正在比较的假设。

## 当前 YAML 怎样打开可视化

`nav2_mppi_baseline.yaml` 中的 `FollowPath` 已包含：

```yaml
visualize: true
TrajectoryVisualizer:
  trajectory_step: 5
  time_step: 3
```

`trajectory_step` 表示显示时跳过部分候选；`time_step` 表示每条候选只显示部分时间点。
它们只减少 RViz 负担，不改变 MPPI 实际评估的候选数量。

调参或 benchmark 时应关掉 `visualize`，因为生成调试 Marker 会增加控制循环开销。

## 如何运行

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup predictive_dynamic_lab_mppi.launch.py
```

发送短距离目标后，检查：

```bash
ros2 topic list | grep -Ei 'mppi|trajectory|cmd_vel'
ros2 topic hz /cmd_vel
```

然后在 RViz 给 Global Path、Local Path 和 MPPI 轨迹设置不同颜色。若某个 topic 名称与预期
不同，以 `ros2 topic list` 的实际输出为准。

## 怎样理解画面

- 候选大多向前但有不同弯曲方向：说明 MPPI 正在探索不同控制序列；
- 大量候选靠近墙时被淘汰：静态 costmap Critic 已在工作；
- `/cmd_vel` 非零而 Gazebo 不动：先检查速度链路或仿真暂停，不要归因于 MPPI；
- action 显示 `aborted`：表示 Nav2 结束了该次目标，不等同于 C++ 节点崩溃。

## 本步完成标准

- [ ] 能在一张截图中区分全局路径、局部路径、候选轨迹和 prediction；
- [ ] 知道候选轨迹的时间步来自 `model_dt` 与 `time_steps`；
- [ ] 知道调试轨迹只用于观察，不能直接当作性能证据；
- [ ] 已在关闭/打开可视化时各观察一次 `/cmd_vel` 频率。

## 下一步

[04_创建mppi插件包骨架](../04_创建mppi插件包骨架/README.md)：开始创建自己的 C++ 共享库，
但此时还不会影响 MPPI。
