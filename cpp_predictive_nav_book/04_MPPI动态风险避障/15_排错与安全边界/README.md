# 15：排错与安全边界

> **本步目标：** 为 prediction 缺失、frame 错误、插件未加载和候选不可行提供可观察的诊断与安全退化。
>
> **AI 会做什么：** 生成诊断日志、检查命令、保护分支和故障注入步骤。
>
> **你只需要做到：** 实际复现至少一个故障，并说明是感知、跟踪预测、critic、MPPI、局部代价地图还是底盘速度链路的问题。

针对预测缺失、插件未加载、frame 错误、时间错位、cost 过大和机器人停滞逐项排查。

本项目的动态 critic 是研究功能，不是认证安全系统；真实机器人仍需要独立的急停、速度限制和静态安全层。

完成标准：每一种失败都有日志、复现方式和明确的安全降级行为。

## 本步：AI 实现诊断与保护，你复现并验证

这一步主要不是写新算法，而是实现并验证故障处理。让 AI 生成诊断与保护分支，至少挑一个故障案例按命令复现、保留日志，并核对系统最后采用的安全行为。

| 故障 | 注入方式 | 你应看到的诊断 | 期望降级 |
|---|---|---|---|
| prediction 停止 | 停掉 prediction_node | age 超阈值 warning | 跳过旧动态 risk；保留 Nav2 静态安全层 |
| frame 错误 | 用测试消息/参数制造非 `odom` frame | frame rejection | 不读取该 prediction |
| 插件未加载 | 临时写错 XML/YAML 名称后恢复 | pluginlib load error | launch 明确失败，不悄悄少一条 Critic |
| cost 异常大 | 临时小 margin / 大权重实验 | cost diagnostics | clamp，避免 NaN/Inf 和无解释卡死 |
| 全部候选不可行 | actor 阻塞窄通道 | MPPI failure / recovery 信息 | 停止或 Nav2 recovery，而不是乱发速度 |

## 必须保留的安全边界

```text
DynamicRiskCritic：研究型连续风险评分，不是认证安全系统
MPPI 静态碰撞检查：仍然存在，不能被替代
scan safety guard / 急停：仍是最终外部底线
真实底盘：必须有速度上限、物理急停和单一 cmd_vel 链路
```

如果发现要在 Critic 里创建 `/cmd_vel` publisher 才能“修好”问题，应停止并回到架构图排查；这会破坏本章的安全边界。

## 故障排查顺序：从链路外层到核心公式

遇到“车不走”时，严格按下面顺序检查并在笔记打勾：

```text
目标 action 是否仍 active？
  ↓
controller_server 是否 active，且确实加载 MPPI？
  ↓
空 Critic 的 baseline 是否仍能走？
  ↓
plugin 是否加载，prediction stamp 是否递增？
  ↓
prediction 是否因 frame / age / finite 被拒绝？
  ↓
dynamic cost 是否有限、是否远大于其他 cost？
  ↓
最后才检查风险公式、权重和 actor 场景
```

不要跳到最后一步直接把 `dynamic_cost_weight` 改成 0；那会掩盖问题。

## AI 应实现的诊断内容

让 AI 在 Critic 内实现节流统计，而不是每个候选、每个时间点打印一行：

```text
received prediction stamp
accepted / rejected prediction frame count
stale frame count
本控制周期最小距离
本控制周期最大单候选 dynamic cost
```

这些统计既能用于调试，也能在面试中说明你考虑过实时控制循环的观测性。你要能从一段日志指出每个统计项代表什么。

真实底盘阶段，不可以把“预测有效”当成安全认证。先在低速度、开放场地、有人可急停的条件下验证；物理急停和独立速度限制必须存在。
