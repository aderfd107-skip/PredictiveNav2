# 08：检查预测时效与坐标系

> 本步目标：在评分前拒绝过期、坐标系错误或包含 NaN/Inf 的 prediction；不合格数据不能偷偷参与风险计算。

## 为什么预测也会过期

prediction 说的是“某个过去时刻对未来的判断”。若当前已过去很久，方块可能已转弯、停下
或离开视野，继续把旧预测当真会制造错误风险。

```text
now - prediction.header.stamp > max_prediction_age_s
                         ↓
                    过期，拒绝评分
```

使用仿真时必须让相关节点使用 `use_sim_time: true`；否则 wall time 与 Gazebo `/clock` 混用，
任何 age 判断都可能不可靠。

## 为什么 frame 必须一致

如果 robot 候选位置是 `odom=(1, 0)`，prediction 却是 `map=(1, 0)`，这两个数值看起来一样，
却不代表地图中的同一点。坐标变换会随 AMCL 校正变化。

本项目要求 prediction 的 `header.frame_id` 为 `odom`，与局部比较约定一致。不符合时记录
拒绝原因；不能只忽略 frame 字符串继续相减。

## 还需要检查哪些内容

| 检查 | 不通过时的做法 |
|---|---|
| 消息是否存在 | 跳过本周期动态风险 |
| header stamp 是否合理 | 拒绝并节流警告 |
| frame 是否为 `odom` | 拒绝该帧 |
| trajectory / poses / offsets 长度是否一致 | 拒绝该轨迹 |
| x、y、协方差是否 finite | 跳过异常轨迹 |

拒绝动态 prediction 并不等于关掉 Nav2。原版静态 costmap Critic、底盘速度约束和 scan safety
guard 仍继续工作；这叫安全退化，不是“旧预测照用”。

## 如何验证

依次测试：正常 prediction、停止 prediction_node、故意发布错误 frame 或异常数值的测试消息。
观察 Critic 日志是否明确显示 accepted/rejected 原因。

不要只看机器人是否移动：机器人能移动不代表旧 prediction 没有误参与评分。

## 本步完成标准

- [ ] 正常新鲜 `odom` prediction 可通过；
- [ ] 过期 prediction 不参与 risk；
- [ ] 非 `odom` frame 不参与 risk；
- [ ] 单个异常目标不会让 controller_server 崩溃；
- [ ] 日志能区分无数据、过期、frame 错误和数值错误。

## 下一步

[09_计算候选轨迹与障碍物距离](../09_计算候选轨迹与障碍物距离/README.md)：只有数据可信后，
才开始遍历 MPPI 候选轨迹与 prediction。
