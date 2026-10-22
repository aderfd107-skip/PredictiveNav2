# 8. 动态风险 MPPI Critic

## 本阶段交付

实现 Nav2 MPPI 的 `mppi::critics::DynamicRiskCritic` 插件，把最新有效预测轨迹以安全、非阻塞的方式接入 MPPI 候选控制轨迹的成本评分。

## 当前实际进度与边界

目前已完成两件事：

1. `DynamicRiskCritic` 已通过 pluginlib 被 Nav2 MPPI 在运行时加载；
2. Critic 已订阅并缓存 `/dynamic_obstacles/predictions` 的最新一帧预测。

当前 `score()` 只读取缓存并输出调试信息，**尚未计算风险，也没有修改 `data.costs`**。因此不能称为“已经实现预测动态避障”；当前准确说法是：**Prediction → MPPI Critic 的数据接入已完成，动态风险评分待后续实现。**

```text
/dynamic_obstacles/predictions
        ↓ ROS subscription callback（生产者）
latest_prediction_ 缓存
        ↓ snapshot
DynamicRiskCritic::score()（消费者）
        ↓ 下一步：时间对齐、风险计算、data.costs(i) += risk
MPPI 在所有 Critic 的总代价中选择控制轨迹
```

## 本章新学知识

- C++ 继承、虚函数与 Nav2 `CriticFunction` 生命周期。
- pluginlib 导出、配置加载、`initialize()`、`score()`。
- `shared_ptr` / `weak_ptr` 的所有权与对象生命周期。
- mutex、RAII 与 snapshot：回调更新预测，控制周期只读最新数据。
- 候选轨迹、时间对齐、距离风险、TTC、安全半径和协方差。
- 控制循环性能：不阻塞、不等待 TF、不频繁分配。

## 关键设计与为什么这样做

### 1. 它是运行时插件，不是普通 C++ 类

Nav2 不会因为源码中存在 `class DynamicRiskCritic` 就自动调用它。实际加载链为：

```text
YAML critics 列表
  → Nav2 CriticManager
  → pluginlib
  → 插件 XML（登记库、类型、基类）
  → shared library
  → PLUGINLIB_EXPORT_CLASS
  → mppi::critics::DynamicRiskCritic
```

三种名称不可混淆：ROS 包和动态库仍叫 `predictive_nav_mppi`；插件 C++ 类型叫 `mppi::critics::DynamicRiskCritic`。Jazzy 的 MPPI `CriticManager` 会按这个命名空间查找 YAML 中列出的 Critic。

### 2. 预测 topic 参数化，而不是写死在代码里

```yaml
DynamicRiskCritic:
  enabled: true
  prediction_topic: "/dynamic_obstacles/predictions"
```

`initialize()` 通过 `ParametersHandler` 读取它，再创建订阅。这样换 topic 或做对照实验只需改 YAML，不用重编译 Critic。

### 3. Subscription 必须是成员变量

`prediction_subscription_` 保存为成员 `SharedPtr`，因为订阅对象本身必须与 Critic 同寿命。若只在 `initialize()` 中创建局部变量，函数结束后订阅会被销毁，callback 不会持续接收预测。

### 4. callback 与 score() 是生产者—消费者并发关系

ROS callback 异步写入最新预测，MPPI 控制循环的 `score()` 同时读取它。两个执行路径都访问同一个 `latest_prediction_`，若无同步会产生 data race。

```text
callback:  写 latest_prediction_
score():  读 latest_prediction_
                 ↑
          prediction_mutex_ 保护 shared_ptr 本身
```

缓存类型是 `PredictionArray::ConstSharedPtr`：Critic 只读消息、不深拷贝整帧预测，并通过共享所有权保证本轮使用的消息仍存活。

### 5. snapshot 缩短临界区

`latest_prediction_snapshot()` 只在锁内复制一个 `ConstSharedPtr`，随后立即解锁；风险计算会在锁外使用这个本地快照。

```text
lock → 复制 shared_ptr → unlock → 对本地快照做高开销评分
```

未来一次评分可能遍历“候选轨迹数 × 时间步 × 动态障碍物数”。若整个 `score()` 始终持锁，callback 将无法及时写入新预测。snapshot 既避免 shared_ptr 读写竞争，又不会让重计算阻塞数据更新。

`std::scoped_lock` 使用 RAII：离开代码作用域时自动解锁，即使中途返回或抛异常也不会遗留锁。`prediction_mutex_` 标为 `mutable`，是因为读取业务上的“最新快照”不改变 Critic 逻辑状态，但 lock/unlock 会改变 mutex 自身状态。

### 6. 不能覆盖 Nav2 原生 Critic 的成本

真正评分时必须累加：

```cpp
data.costs(i) += dynamic_risk;
```

不能赋值：

```cpp
data.costs(i) = dynamic_risk;
```

后者会抹掉 `BaseObstacle`、路径跟随、目标距离等原生 Critic 的成本，使 MPPI 不再按完整目标优化。

## 面试时可直接讲的版本

> 我的项目用 2D LiDAR 聚类得到障碍物观测，经过多目标跟踪和 CV 预测后发布未来轨迹。为了把预测结果接入 Nav2 MPPI，我实现了一个自定义 `DynamicRiskCritic` plugin，通过 pluginlib 由 YAML 在运行时加载。预测 callback 缓存最新消息，MPPI 的 `score()` 消费这个缓存。两者可能并发访问，所以我用 mutex 保护 shared_ptr，并采用 snapshot：锁内只复制 `ConstSharedPtr`，立即释放锁，后续风险计算在锁外完成，避免阻塞控制路径。当前已经完成预测数据接入；下一步会校验 frame 和时效、对齐候选轨迹与障碍物预测的时间，再以 `data.costs += risk` 叠加动态风险，而不覆盖 Nav2 原生成本。

## 当前设计约束

- 保持 `nav2_mppi_baseline.yaml` 不变，作为原版 MPPI 对照组；
- `prediction_topic` 保持 YAML 参数化；
- 不深拷贝预测消息，不在 mutex 临界区做风险循环；
- 在输入通过 frame、时间戳、时效、轨迹和协方差合法性检查前，不影响 MPPI 成本；
- 未完成 `data.costs(i) += risk` 前，不把当前实现表述为动态风险避障。

## 实施顺序

先实现“能被加载且不改变成本”的空 Critic；再读取并缓存预测（当前完成）；接着检查 frame/时效/数值合法性；再实现距离风险；最后引入时间对齐、TTC 与不确定性。

## 完成标准

- 当前验收：插件稳定加载、收到预测后能输出缓存日志，且控制循环不出现明显卡顿或崩溃；
- 最终验收：动态 actor 使相关候选轨迹的**累计成本**升高，并能在固定场景与原版 MPPI 对照验证效果。
