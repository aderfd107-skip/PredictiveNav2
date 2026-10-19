# 12：配置参数与权重

> **本步目标：** 将风险半径、预测时效、TTC 和权重变成可在 YAML 中调节的参数，而不是散落在 C++ 中的数字。
>
> **AI 会做什么：** 根据本项目底盘、地图和 prediction horizon 生成有单位的起始参数、范围检查和配置摘要日志。
>
> **你只需要做到：** 能回答某个参数增大/减小时风险偏好如何改变，并一次只改一个参数验证预期。

把风险半径、时间 horizon、最大预测年龄、权重和保守策略放入 YAML 参数，不写死在 C++。

**权重**决定动态风险在所有 critic 中的重要程度；它不是越大越安全，过大会导致机器人无故停住。

完成标准：能只改 YAML 做一次可解释的参数实验，并记录改动原因。

## 本步：让 AI 参数化，你审查单位和范围

让 AI 把风险公式中的“魔法数字”变成插件参数，并在 `initialize()` 中验证范围。你要审查每个参数的名称、默认值、单位、范围与行为影响，而不是手抄 YAML 或样板 C++。

## 推荐的第一批参数

| 参数 | 单位 | 作用 | 最低校验 |
|---|---:|---|---|
| `enabled` | - | 是否启用 Critic | bool |
| `max_prediction_age_s` | s | 最旧可接受 prediction | `> 0` |
| `robot_radius_m` | m | footprint 的保守等效半径 | `> 0` |
| `base_safety_margin_m` | m | 基础余量 | `>= 0` |
| `sigma_scale` | - | 协方差保守系数 | `>= 0` |
| `influence_radius_m` | m | 开始加距离风险的范围 | `> safe radius` |
| `dynamic_cost_weight` | - | 动态风险权重 | `>= 0` |
| `ttc_horizon_s` | s | TTC 关注窗口 | `> 0` |
| `ttc_cost_weight` | - | TTC 风险权重 | `>= 0` |
| `collision_cost` | - | 极近距离的单点上限 | `> 0` |

## 实际 API 提示

基类已提供 `parameters_handler_` 与当前 Critic 名称 `name_`。在 `initialize()` 中先理解并使用：

```cpp
auto get_param = parameters_handler_->getParamGetter(name_);
get_param(dynamic_cost_weight_, "dynamic_cost_weight", 1.0F);
```

让 AI 补完其余参数和范围检查。检查它没有在 `score()` 中反复读取参数；控制循环应只读已验证的成员变量。

## 必做的小实验

固定同一 bag / 起点 / 目标，只改一个参数：

```text
dynamic_cost_weight: 1.0 → 2.0
```

记录最小距离、完成时间、停顿次数和是否出现无故停车。若只说“更安全了”，但没有指标或代价说明，这不算完成。

## 推荐流程：由 AI 生成，按一个参数一次地验证

让 AI 在头文件中为每个参数创建带单位的成员，例如：

```cpp
double max_prediction_age_s_{0.5};
double robot_radius_m_{0.135};
double base_safety_margin_m_{0.10};
double dynamic_cost_weight_{1.0};
```

让 AI 在 `initialize()` 中逐个读取 YAML。审查每个参数是否立即校验范围；不合法时不可默默改成另一个值，必须给出包含插件名和参数名的错误日志。最后应将已接受的参数一次性打印成摘要。

推荐你按这个次序放开参数：

```text
enabled
max_prediction_age_s
robot_radius_m + base_safety_margin_m
influence_radius_m
dynamic_cost_weight
sigma_scale
TTC 相关参数
```

这样某次行为变化时，你知道是哪个概念造成的，而不是面对十个一起变的数字猜答案。

## YAML 模板

让 AI 先按当前机器人和地图给出带理由的起始值；你应在每个数值旁确认单位和选择理由：

```yaml
FollowPath:
  DynamicRiskCritic:
    enabled: true
    max_prediction_age_s: TODO
    robot_radius_m: TODO
    base_safety_margin_m: TODO
    influence_radius_m: TODO
    dynamic_cost_weight: TODO
```

让 AI 根据最终代码生成“参数名 → 对应代码成员 → 单位 → 影响什么行为”的表；你核对后保存。能维护参数边界，比记住 YAML 缩进更重要。
