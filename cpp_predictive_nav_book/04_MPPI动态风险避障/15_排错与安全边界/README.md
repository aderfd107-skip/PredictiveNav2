# 15：排错与安全边界

> 本步目标：让预测链路发生故障时系统可诊断、可安全退化，而不是悄悄使用错误数据或乱发速度。

## DynamicRiskCritic 不是什么

它是一个研究型、连续评分模块，不是认证的安全系统：

```text
DynamicRiskCritic     动态预测风险偏好
MPPI 静态碰撞检查     原有局部可行性与静态障碍保护
scan_safety_guard     当前 scan 的外层速度/停车底线
真实底盘急停          独立物理安全措施
```

任何情况下都不要为了“修好避障”而在 Critic 里新增 `/cmd_vel` publisher。这会破坏单一
控制链路，也会绕过 Nav2 与外部安全层。

## 需要能复现的故障

| 故障 | 如何注入 | 期望表现 |
|---|---|---|
| prediction 停止 | 停掉 prediction_node | age 超阈值，跳过动态 risk |
| frame 错误 | 测试消息使用非 `odom` frame | 该帧被拒绝 |
| 插件未加载 | 临时写错 XML/YAML 后恢复 | 明确 pluginlib 错误 |
| cost 异常大 | 极端协方差或权重 | clamp 和诊断，不出现 NaN/Inf |
| 通道全部不可行 | actor 阻塞窄通道 | 停止或 Nav2 recovery，不乱发速度 |

## “车不走”时的检查顺序

```text
目标 action 仍 active 吗？
  ↓
controller_server active 且正在使用 MPPI 吗？
  ↓
baseline MPPI 能走吗？
  ↓
Critic 已加载，prediction stamp 在更新吗？
  ↓
prediction 是否因 frame / age / finite 被拒绝？
  ↓
dynamic cost 是否有限且没有压倒所有其他 cost？
  ↓
最后才检查风险公式、参数与 actor 场景
```

不要一开始把 `dynamic_cost_weight` 设为 0；那会掩盖真实故障。

## 推荐诊断统计

不要在每条候选、每个时间点打印一行。使用节流日志汇总：

```text
最近 prediction stamp
accepted / rejected frame 数
stale frame 数
本周期最小动态距离
本周期最大单候选 dynamic cost
```

这些信息能让你在日志中区分感知、tracking/prediction、Critic、MPPI 或底盘链路的问题。

## 本步完成标准

- [ ] 至少复现一个 prediction 故障；
- [ ] 系统拒绝无效预测但保留静态安全层；
- [ ] 没有 NaN/Inf 或无解释的速度发布；
- [ ] 日志足以指出故障位于哪一层；
- [ ] 知道真机仍需要独立急停与速度限制。

## 下一步

[16_对比原版MPPI基线](../16_对比原版MPPI基线/README.md)：冻结实验条件，量化动态风险
带来的收益和代价。
