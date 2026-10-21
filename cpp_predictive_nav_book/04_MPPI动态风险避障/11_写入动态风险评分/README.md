# 11：写入动态风险评分

> 本步目标：把距离、预测不确定性和 TTC 合成连续、有限、非负的动态风险，并累加到每条 MPPI 候选轨迹的 cost。

## cost 在 MPPI 中是什么意思

cost 只是“这条候选有多不想选”的分数，不是物理能量。MPPI 会综合所有 Critic 的 cost，
更高动态风险会让候选相对不受偏好。

第一版不追求复杂论文公式，而追求能解释、连续和数值稳定。

## 安全半径怎样考虑不确定性

```text
safe_radius = robot_radius
            + obstacle_radius
            + base_safety_margin
            + sigma_scale × sqrt(max_eigenvalue(covariance))
```

协方差大时，最后一项变大，表示预测越不可靠，机器人与该目标保持的保守距离越大。
这里不是说障碍物真的变大，而是控制决策对不确定性更谨慎。

## 距离风险应满足什么性质

```text
distance >= influence_radius     → risk = 0
distance <= safe_radius          → risk = collision_cost
两者之间                       → 连续地从 0 增至 collision_cost
```

中间可以先使用平方或线性曲线。无论选哪一种，都必须保证所有有限输入得到有限且不小于
0 的输出，避免异常协方差把 `data.costs` 变成 NaN/Inf。

## TTC 风险怎样加入

距离接近表示“空间上危险”，TTC 小表示“很快就会相遇”。TTC risk 使用独立权重，不能把它
混成一个无法解释的常数。相同距离下，TTC 为 0.5 s 的候选应比 TTC 为 3 s 的候选更危险。

## 怎样写入 `data.costs`

```text
for 每条候选 i:
  candidate_risk = 0
  for 每个有效时间点和障碍物:
    candidate_risk += bounded_distance_risk + bounded_ttc_risk
  data.costs(i) += dynamic_cost_weight × candidate_risk
```

不能使用 `data.costs(i) = ...`，因为这会覆盖 Nav2 内置 Critic 的结果。单点风险和整条
轨迹风险都应有上限，以免单帧异常主导全部候选。

## 三个最小测试候选

| 候选 | 最近距离 | 预测方差 | TTC | 预期风险 |
|---|---:|---:|---:|---|
| A：远离 | 2.0 m | 小 | 无 | 0 或很小 |
| B：擦边 | 0.30 m | 小 | 1.0 s | 高 |
| C：同样擦边但不确定 | 0.30 m | 大 | 1.0 s | 不低于 B |

先比较测试输出排序，再观察实际日志。最终“没有撞”不能单独证明 risk 公式正确。

## 本步完成标准

- [ ] 距离更小、TTC 更小或协方差更大时，risk 不降低；
- [ ] 足够远时 risk 为 0；
- [ ] 无可用 prediction 时不改变 `data.costs`；
- [ ] 所有有限输入都不会产生 NaN/Inf；
- [ ] 只用 `+=` 累加自己的动态风险。

## 下一步

[12_配置参数与权重](../12_配置参数与权重/README.md)：将公式中的半径、时间窗口和权重
移到 YAML，避免散落的魔法数字。
