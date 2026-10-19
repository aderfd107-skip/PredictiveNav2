# 13：在仿真中启用 critic

> **本步目标：** 在专用仿真配置中启用 DynamicRiskCritic，同时永久保留可回退的原版 MPPI baseline。
>
> **AI 会做什么：** 生成 baseline/dynamic-risk YAML 差分与 launch 参数，并给出检查插件是否真正加载的命令。
>
> **你只需要做到：** 核对实验配置只增加动态风险 critic，没有悄悄改变机器人半径、地图、安全层或速度边界。

将新 critic 加入专用动态实验配置，而不是覆盖原版 baseline；启动后检查插件、prediction topic 和 Nav2 lifecycle 都正常。

**专用配置**确保你随时能退回原版对照组，不会因实验参数污染日常导航。

完成标准：同一场景可切换“原版 MPPI”和“MPPI + DynamicRiskCritic”。

## 本步：让 AI 生成可回退配置，你验证差异

不新增核心 C++ 算法。让 AI 生成两份配置与启动入口；你要用 diff 核对 baseline 始终可回退，动态版只增加 DynamicRiskCritic 及其参数。这是工程能力，不是小事。

## AI 应创建的内容

```text
nav2_mppi_baseline.yaml             原版 MPPI，不含你的 critic
nav2_mppi_dynamic_risk.yaml         原版 MPPI + DynamicRiskCritic
```

两份文件除 controller 和 DynamicRiskCritic 参数外，地图、AMCL、footprint、速度限制、goal checker、costmap 必须一致。

在动态版本 YAML 中，AI 应加入：

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

## 推荐流程：生成两个可回退的实验入口

让 AI 创建并维护下面这对文件：

```text
nav2_mppi_baseline.yaml       原版 MPPI；不能出现你的插件
nav2_mppi_dynamic_risk.yaml   仅在 baseline 基础上加入你的插件与参数
```

用 `diff -u` 检查它们。除 `FollowPath` 的 controller/critic 配置外，不应有地图、机器人尺寸、速度上限、costmap 或 AMCL 参数差异：

```bash
diff -u \
  src/predictive_nav_bringup/config/nav2_mppi_baseline.yaml \
  src/predictive_nav_bringup/config/nav2_mppi_dynamic_risk.yaml
```

再让 AI 创建一个 MPPI 专用 launch 入口，或为现有 launch 增加清晰的 `params_file` 参数。你要能回答：本次启动日志中哪一行能证明加载的是 MPPI YAML，而不是旧 `nav2_dwb.yaml`。

### 启动时的排查顺序

```text
1. baseline MPPI 无插件：能启动、能走
2. baseline MPPI + 空 DynamicRiskCritic：行为应基本一致
3. prediction_node 启动：Critic 收到递增 stamp
4. 动态风险公式开启：才允许观察行为变化
```

任何一步失败，回退到上一步。不要同时改 C++、YAML、动态 actor 路线和 cost 权重。

## 本节交付物

两份 YAML、launch 中参数文件的传递位置、`diff -u` 输出、plugin 成功加载日志，以及一次“空 Critic 前后行为一致”的记录。
