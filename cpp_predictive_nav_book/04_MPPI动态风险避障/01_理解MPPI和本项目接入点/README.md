# 01：理解 MPPI 和本项目接入点

先用项目现有导航链路解释 MPPI 的位置：全局规划给方向，MPPI 在局部不断挑选短时间驾驶轨迹。

**critic（评价器）**是 MPPI 的一个可插拔评分规则。本项目只新增动态风险这一条规则，不重写 Nav2 控制器。

完成标准：能画出 `/predictions → critic → MPPI → /cmd_vel`，并知道 critic 不应直接控制小车。
