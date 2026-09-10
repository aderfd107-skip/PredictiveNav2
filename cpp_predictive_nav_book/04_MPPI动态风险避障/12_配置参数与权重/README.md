# 12：配置参数与权重

把风险半径、时间 horizon、最大预测年龄、权重和保守策略放入 YAML 参数，不写死在 C++。

**权重**决定动态风险在所有 critic 中的重要程度；它不是越大越安全，过大会导致机器人无故停住。

完成标准：能只改 YAML 做一次可解释的参数实验，并记录改动原因。

## 本步必须手写

你要亲自把风险公式中的“魔法数字”变成插件参数，并在 `initialize()` 中验证范围。YAML 格式可以参考，但参数名、默认值、单位和校验逻辑要由你写。

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

然后自己补完其余参数和范围检查。不要在 `score()` 中反复读取参数；控制循环要只读已验证的成员变量。

## 必做的小实验

固定同一 bag / 起点 / 目标，只改一个参数：

```text
dynamic_cost_weight: 1.0 → 2.0
```

记录最小距离、完成时间、停顿次数和是否出现无故停车。若只说“更安全了”，但没有指标或代价说明，这不算完成。
