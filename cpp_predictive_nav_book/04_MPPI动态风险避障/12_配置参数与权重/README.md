# 12：配置参数与权重

> 本步目标：把动态风险公式中的数字变成有单位、有范围检查、可记录的 YAML 参数。

## 为什么不能把数字写死在 C++

机器人半径、预测刷新率、地图通道宽度和风险偏好都会变化。若 `0.35`、`2.0`、`50.0`
散落在 `score()` 内，别人无法知道它们代表什么，也无法只改配置完成实验。

参数应在 `initialize()` 中读取并校验；`score()` 只使用已经验证的成员变量，避免在 20 Hz
控制循环中反复查询 ROS 参数。

## 推荐的第一批参数

| 参数 | 单位 | 作用 |
|---|---:|---|
| `enabled` | - | 是否启用 DynamicRiskCritic |
| `max_prediction_age_s` | s | 允许使用的最大 prediction 年龄 |
| `robot_radius_m` | m | 机器人保守等效半径 |
| `base_safety_margin_m` | m | 不依赖协方差的额外余量 |
| `sigma_scale` | - | 协方差转为安全半径的系数 |
| `influence_radius_m` | m | 从多远开始产生距离风险 |
| `dynamic_cost_weight` | - | 动态风险在 MPPI 中的总权重 |
| `ttc_horizon_s` | s | TTC 关注范围 |
| `ttc_cost_weight` | - | TTC 风险权重 |
| `collision_cost` | - | 极近距离单点风险上限 |

## 参数范围为什么重要

```text
robot_radius_m > 0
base_safety_margin_m >= 0
max_prediction_age_s > 0
influence_radius_m > safe_radius
dynamic_cost_weight >= 0
collision_cost > 0
```

不合法时应明确报错，不能悄悄替换为另一个数值。否则实验记录中的 YAML 与实际控制行为
可能不一致。

## YAML 的位置

参数属于 MPPI 的 `FollowPath` 与该 Critic：

```yaml
FollowPath:
  critics: ["...", "DynamicRiskCritic"]
  DynamicRiskCritic:
    enabled: true
    max_prediction_age_s: 0.5
    robot_radius_m: 0.135
    base_safety_margin_m: 0.10
    influence_radius_m: 0.80
```

数值只是起点，不是通用最优答案。每次实验只改一个参数，并记录最小距离、完成时间、
停顿和 recovery，而不是只凭“看起来更安全”。

## 本步完成标准

- [ ] 每个参数有名称、单位、默认值和范围；
- [ ] `score()` 不读取 ROS 参数；
- [ ] 不合法配置会明确失败；
- [ ] 能解释增大 `dynamic_cost_weight` 的收益和代价；
- [ ] 参数摘要能写入实验记录。

## 下一步

[13_在仿真中启用critic](../13_在仿真中启用critic/README.md)：为 baseline 与动态风险版
保留两份独立配置，开始真正加载和运行。
