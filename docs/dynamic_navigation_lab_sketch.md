# Predictive Dynamic Lab 场景说明

新的静态基线世界为 `predictive_dynamic_lab.sdf`，尺寸 `20 m × 16 m`，不含任何动态 actor。
它包含开放区、十字横穿区、迎面走廊、盲区 L 转角、T 字路口、90° 转角和 `0.70 m`
临时设计宽度的窄通道。启动入口是 `predictive_dynamic_lab_static.launch.py`；mapping launch
也默认使用该世界。

先在此世界手动驱动机器人，检查碰撞、LiDAR 遮挡、通道宽度和可达性，并保存新的静态地图。只有
静态 Nav2/MPPI 基线稳定后，才在单独 launch 中引入一个可复现 actor。actor 的真值轨迹不能直接
提供给 tracking、prediction 或 DynamicRiskCritic。
