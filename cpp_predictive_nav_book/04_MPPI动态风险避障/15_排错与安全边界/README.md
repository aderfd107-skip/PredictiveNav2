# 15：排错与安全边界

针对预测缺失、插件未加载、frame 错误、时间错位、cost 过大和机器人停滞逐项排查。

本项目的动态 critic 是研究功能，不是认证安全系统；真实机器人仍需要独立的急停、速度限制和静态安全层。

完成标准：每一种失败都有日志、复现方式和明确的安全降级行为。

## 本步必须亲自完成的内容

这一步主要不是写新算法，而是写出并验证故障处理。下面每项都要至少复现一次、保留日志，并写明系统最后采用什么安全行为。

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
