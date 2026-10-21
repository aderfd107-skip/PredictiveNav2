# 05：定义 DynamicRiskCritic 插件

> 本步目标：定义一个符合 Nav2 MPPI Critic 接口的空类；它可以编译，但仍未被 Nav2 加载，也不改变候选轨迹 cost。

## 为什么需要这个类

第 04 步只有一个 `.so` 库，Nav2 不知道库中哪一个 C++ 类是 Critic。现在新增：

```cpp
class DynamicRiskCritic : public mppi::critics::CriticFunction
```

实现时类会放在 `mppi::critics` namespace 中。这是 Jazzy MPPI 加载器的命名约定；
它不表示代码属于 Nav2 包，工程包仍是 `predictive_nav_mppi`。

继承 `CriticFunction` 的意思是接受 Nav2 规定的两件事：初始化时调用 `initialize()`，每次
MPPI 评分候选轨迹时调用 `score(mppi::CriticData & data)`。

## 头文件中的两个函数

```cpp
void initialize() override;
void score(mppi::CriticData & data) override;
```

`override` 要求函数签名和 Nav2 基类完全一致。少写引用、写错命名空间或参数类型都会在
编译期报错；这比运行后才发现 Critic 从未被调用安全得多。

`CriticData` 是 MPPI 每个控制周期传入的上下文，其中最重要的是：

```text
data.trajectories.x / y / yaws   候选机器人轨迹，约为 [batch, time]
data.costs                       每条候选轨迹的累计代价
data.model_dt                    候选轨迹的时间步长
```

## 当前实现做了什么

`initialize()` 从基类保存的 lifecycle node 获取 ROS clock，并打印：

```text
Initialized <插件名> as an empty MPPI critic (enabled: true/false)
```

`enabled_` 由 Nav2 基类从该 Critic 的 YAML 参数读取。它为 false 时，`score()` 立即返回。

`score()` 当前只读取 `data.trajectories.x.shape()`，每 2 秒最多输出一次候选条数和时间步数。
这条日志帮助确认以后 Critic 的确在 MPPI 控制循环中被调用。

## 为什么现在绝不能写 `data.costs`

当前空 Critic 的目的只是验证 C++ 接口。若现在修改 `data.costs`，后面一旦导航行为变化，
无法判断是 plugin 加载、数组维度、风险公式还是参数导致。

后续真正评分时会使用：

```cpp
data.costs(i) += dynamic_risk_cost;
```

而不是赋值。这样不会擦掉 Nav2 的静态障碍、路径跟随和目标 Critic 已经计算的代价。

## 如何构建

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select predictive_nav_mppi --symlink-install
```

本步只要求编译成功；因为还没有 pluginlib 导出，所以启动 Nav2 后不会看到初始化日志。
这是正常现象，不是代码失效。

## 本步完成标准

- [ ] `DynamicRiskCritic` 继承 `mppi::critics::CriticFunction`；
- [ ] 两个 `override` 函数编译通过；
- [ ] `score()` 没有修改 `data.costs`；
- [ ] 没有 prediction subscription、mutex、距离或 TTC 代码；
- [ ] 知道下一步必须导出该类，Nav2 才会调用它。

## 下一步

[06_导出并加载插件](../06_导出并加载插件/README.md)：用 pluginlib XML、导出宏和 YAML
把空 Critic 真正交给 controller_server。
