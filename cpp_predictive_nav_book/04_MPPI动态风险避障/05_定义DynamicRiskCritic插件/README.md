# 05：定义 DynamicRiskCritic 插件

创建继承 Nav2 MPPI critic 基类的 `DynamicRiskCritic`，只实现生命周期入口和空评分函数。

**继承**表示新类复用 Nav2 规定的接口；它不是复制粘贴控制器代码。空评分阶段先证明接口正确，再逐项增加逻辑。

完成标准：代码能说明初始化、评分和清理分别在哪发生，且空 critic 不改变结果。

## 本步必须手写

这是本章第一个核心 C++ 练习。请先自己写头文件中的类声明和 `.cpp` 中两个空函数；不要先向 AI 要完整实现。

### 需要的真实 Jazzy 头文件

```cpp
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
```

### 你要补出的类骨架

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

## 先写伪代码，再写函数体

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

本步只观察它们，不修改张量、不写距离公式。写完后把你的头文件和 cpp diff 发给我；我会先检查继承、namespace 和函数签名。
