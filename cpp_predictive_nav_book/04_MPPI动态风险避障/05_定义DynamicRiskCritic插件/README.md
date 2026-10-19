# 05：定义 DynamicRiskCritic 插件

> **本步目标：** 生成一个空的 `DynamicRiskCritic`，先证明它遵守 Nav2 MPPI 的插件接口；此时它不改变任何候选轨迹的代价。
>
> **AI 会做什么：** 生成类声明、`initialize()` 和 `score()` 的首版，并解释继承、`override` 和 `CriticData`。
>
> **你只需要做到：** 能解释 `initialize()` 只做一次性准备、`score()` 在每轮 MPPI 评估中调用，以及 critic 为什么只能给候选轨迹加代价。

创建继承 Nav2 MPPI critic 基类的 `DynamicRiskCritic`，只实现生命周期入口和空评分函数。

**继承**表示新类复用 Nav2 规定的接口；它不是复制粘贴控制器代码。空评分阶段先证明接口正确，再逐项增加逻辑。

完成标准：代码能说明初始化、评分和清理分别在哪发生，且空 critic 不改变结果。

## 本步 AI 生成与审读

这是本章第一个核心 C++ 结构。AI 生成头文件中的类声明和 `.cpp` 中两个空函数；你要确认继承的基类、两个 `override` 函数和“空 critic 不修改 cost”的边界。

### 需要的真实 Jazzy 头文件

```cpp
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
```

### AI 应生成的类骨架

```cpp
namespace predictive_nav_mppi
{

class DynamicRiskCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;
};

}  // namespace predictive_nav_mppi
```

`override` 不是装饰：它要求函数签名必须与 Nav2 基类一致，拼错参数类型会在编译期暴露。

## 先读伪代码，再检查函数体

```text
initialize:
  从基类提供的 name_ / parameters_handler_ 读取本插件参数
  只打印一次“插件已初始化”日志

score:
  若 enabled_ 为 false，立即返回
  暂时不修改 data.costs
  只节流打印候选轨迹数量和时间步数
```

`score()` 的空版本不能向 `data.costs` 加任何数；这样第 06 步加载后，原版 MPPI 和带空 Critic 的行为应一致。

## 你需要读懂的对象

`mppi::CriticData` 至少包含：

```text
data.trajectories.x / y / yaws   候选轨迹张量，形状约为 [batch, time_step]
data.costs                       每条候选轨迹的累计代价
data.model_dt                    MPPI 候选时间步长
data.path / data.goal            全局路径和局部目标
```

本步只观察它们，不修改张量、不写距离公式。生成后查看头文件和 cpp diff，确认继承、namespace 和函数签名。

## 建议的 AI 生成与验证顺序

让 AI 按下面顺序生成，并在每一项后编译一次。这样你知道错误来自哪一项，而不是只得到一段无法定位的问题代码。

1. 头文件只写 include guard、两个 Nav2 头文件、namespace、类声明；
2. `.cpp` 只 include 自己的头文件，写空的 `initialize()` 与 `score()`；
3. 在 `initialize()` 中用 `RCLCPP_INFO(logger_, ...)` 打印 `name_`；
4. 在 `score()` 第一行写 `if (!enabled_) { return; }`；
5. 只读取并节流打印 `data.trajectories.x.shape()[0]` 与 `shape()[1]`，不要写 `data.costs`。

本机 Jazzy 的真实基类接口是：

```cpp
virtual void initialize() = 0;
virtual void score(mppi::CriticData & data) = 0;
```

因此这两个函数的参数、返回类型和 `override` 必须完全一致。若编译器报“does not override”，先检查函数签名，不能删除 `override` 来掩盖错误。

## 你现在不该写的内容

```text
订阅 prediction topic        第 07 节
mutex / 最新消息快照         第 07 节
frame / age 校验             第 08 节
距离、TTC、协方差公式        第 09–11 节
PLUGINLIB_EXPORT_CLASS       第 06 节
```

空 Critic 的价值是隔离问题：如果它加载后机器人行为已改变，先查插件、参数、MPPI baseline；不要把问题误认为风险算法。
