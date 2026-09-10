# 13：在仿真中启用 critic

将新 critic 加入专用动态实验配置，而不是覆盖原版 baseline；启动后检查插件、prediction topic 和 Nav2 lifecycle 都正常。

**专用配置**确保你随时能退回原版对照组，不会因实验参数污染日常导航。

完成标准：同一场景可切换“原版 MPPI”和“MPPI + DynamicRiskCritic”。

## 本步手写范围

不新增核心 C++ 算法。你要亲自维护两份配置与启动入口，保证 baseline 永远能回退；这是工程能力，不是小事。

## 你要亲自创建的内容

```text
nav2_mppi_baseline.yaml             原版 MPPI，不含你的 critic
nav2_mppi_dynamic_risk.yaml         原版 MPPI + DynamicRiskCritic
```

两份文件除 controller 和 DynamicRiskCritic 参数外，地图、AMCL、footprint、速度限制、goal checker、costmap 必须一致。

在动态版本 YAML 中，你要自己加入：

```yaml
FollowPath:
  critics: ["...原有 critic...", "DynamicRiskCritic"]
  DynamicRiskCritic:
    enabled: true
    # 第 12 步的参数
```

## 启动前检查清单

- build 后是否 `source install/setup.bash`；
- launch 是否真的使用 dynamic-risk YAML，而不是旧 DWB/YAML；
- `prediction_node` 是否运行且 `use_sim_time:=true`；
- controller_server lifecycle 是否为 `active`；
- 日志是否明确显示 `DynamicRiskCritic` 已加载。

若车不动，先在**关闭 Critic**的 MPPI baseline 复现；不要一上来就修改风险公式。只有 baseline 正常、插件加载正常、prediction 新鲜，才有资格调权重。
