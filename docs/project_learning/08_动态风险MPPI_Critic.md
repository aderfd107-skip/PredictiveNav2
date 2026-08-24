# 8. 动态风险 MPPI Critic

## 本阶段交付

实现 `predictive_nav_mppi::DynamicRiskCritic`，把最新有效预测轨迹加入 MPPI 候选控制轨迹的成本评分。

## 本章新学知识

- C++ 继承、虚函数与 Nav2 `CriticFunction` 生命周期。
- pluginlib 导出、配置加载、`initialize()`、`score()`。
- 候选轨迹、时间对齐、距离风险、TTC、安全半径和协方差。
- mutex/快照缓存：回调更新预测，控制周期只读最新数据。
- 控制循环性能：不阻塞、不等待 TF、不频繁分配。

## 实施顺序

先实现“能被加载且不改变成本”的空 Critic；再读取预测；再只实现距离风险；最后引入 TTC 与不确定性。

## 完成标准

插件能稳定加载；动态 actor 使相关候选轨迹成本升高；控制循环不出现明显卡顿或崩溃。
