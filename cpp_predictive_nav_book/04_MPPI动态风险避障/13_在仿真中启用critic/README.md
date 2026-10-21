# 13：在仿真中启用 Critic

> 本步目标：在不污染原版 MPPI baseline 的前提下，启动 MPPI + DynamicRiskCritic 的完整仿真链路。

## 为什么要保留两份 YAML

应始终保留：

```text
nav2_mppi_baseline.yaml       原版 MPPI
nav2_mppi_dynamic_risk.yaml   原版 MPPI + DynamicRiskCritic
```

两份文件除 `FollowPath.critics` 与 DynamicRiskCritic 参数外，地图、footprint、速度上限、
costmap、AMCL、goal checker 都应一致。否则实验行为变化可能来自另一个参数，无法归因于
动态风险。

使用下面命令检查：

```bash
diff -u \
  src/predictive_nav_bringup/config/nav2_mppi_baseline.yaml \
  src/predictive_nav_bringup/config/nav2_mppi_dynamic_risk.yaml
```

## 启动顺序

```text
1. baseline MPPI：确认能走
2. baseline + 空 Critic：行为应基本不变
3. prediction_node：确认 stamp 持续更新
4. 启用动态风险公式：才观察减速、等待或绕行
```

一次只改变一层。不要同时改 C++、YAML、actor 轨迹和 risk 权重，否则无法定位问题。

## 启动后要确认什么

| 项目 | 预期 |
|---|---|
| controller_server | lifecycle 为 active |
| DynamicRiskCritic | 日志显示 initialized / loaded |
| prediction_node | `/dynamic_obstacles/predictions` 持续发布 |
| MPPI | `/cmd_vel` 有合理频率 |
| Gazebo | actor 与机器人都在正确地图中 |

若车不动，第一步永远是退回 baseline 验证，而不是马上把动态权重调小。

## 本步完成标准

- [ ] baseline 与动态风险 YAML 可以独立启动；
- [ ] 两份配置差异可解释；
- [ ] controller_server 明确加载 DynamicRiskCritic；
- [ ] 空 Critic 不改变 baseline 行为；
- [ ] 预测、MPPI、Gazebo 和 RViz 同时处于工作状态。

## 下一步

[14_观察减速等待绕行](../14_观察减速等待绕行/README.md)：在固定动态场景中观察行为，
并保留可比较的证据。
