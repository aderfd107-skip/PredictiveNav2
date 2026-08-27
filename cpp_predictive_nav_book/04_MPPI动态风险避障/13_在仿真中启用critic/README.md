# 13：在仿真中启用 critic

将新 critic 加入专用动态实验配置，而不是覆盖原版 baseline；启动后检查插件、prediction topic 和 Nav2 lifecycle 都正常。

**专用配置**确保你随时能退回原版对照组，不会因实验参数污染日常导航。

完成标准：同一场景可切换“原版 MPPI”和“MPPI + DynamicRiskCritic”。
