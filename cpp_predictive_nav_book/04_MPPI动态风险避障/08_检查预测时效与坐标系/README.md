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

## 本工程的实现

第 07 步的 callback 仍然只做两件事：收到消息后替换 `latest_prediction_`，并输出收到日志。
它不在 callback 内决定消息能否参与控制，因为“是否过期”取决于 **当前** MPPI 控制周期的
ROS 时间。

本步由 `score()` 在拿到 snapshot 后依次执行：

```text
没有缓存
  → 跳过动态部分

有缓存
  → header.frame_id 是否为 expected_prediction_frame（当前 odom）
  → header.stamp 是否为正、未过期、未异常地来自未来
  → 每条 trajectory 的 poses / time_offsets 长度是否一致
  → 每个 offset 是否递增且合法
  → pose 的 x、y 与 2D 协方差是否都是有限数
  → 本步只记录“可用/拒绝”，仍不修改 data.costs
```

动态 MPPI 配置中的参数为：

```yaml
DynamicRiskCritic:
  prediction_topic: "/dynamic_obstacles/predictions"
  expected_prediction_frame: "odom"
  max_prediction_age_s: 0.50
  max_future_time_skew_s: 0.10
```

`max_prediction_age_s: 0.50` 不是“预测未来 0.5 秒”，而是允许消息距当前 ROS 时间最多
0.5 秒。`max_future_time_skew_s: 0.10` 则容忍很小的时钟调度差，但拒绝明显晚于当前 `/clock`
的时间戳。

这里仍保留第 07 步的短临界区：mutex 只保护 shared pointer 的替换和复制；逐条检查在锁外
完成。这样未来的距离/TTC 风险循环不会阻塞 prediction callback。

## 如何验证

先使用第 07 步的多终端启动方式，启动 MPPI、perception、tracking 和 prediction。然后看
`controller_server` 的终端：

正常新鲜的 `odom` 消息会输出类似：

```text
DynamicRiskCritic accepted prediction frame: ... usable trajectory(s), age=0.0xx s; scoring remains unchanged
```

若停止 `prediction_node`，等待超过 0.5 秒后，最新缓存会输出：

```text
DynamicRiskCritic rejected cached prediction frame | reason=timestamp is older than max_prediction_age_s
```

这说明 Critic **不会**继续把旧 prediction 当作当前动态障碍物；但 Nav2 仍会按原生静态
Critics 工作。

错误 frame 和异常数值的注入测试将在下一轮专门构造测试 publisher 时完成；本步代码已经会
分别输出 `frame does not match expected_prediction_frame`、`pose and time_offsets sizes differ`、
`pose x/y or 2D position covariance is non-finite` 等原因。

不要只看机器人是否移动：机器人能移动不代表旧 prediction 没有误参与评分。

## 本步完成标准

- [x] 为 Critic 加入可参数化的 `odom`、时效和未来时间偏差检查；
- [x] 为每条轨迹加入长度、时间 offset、x/y 和二维协方差有限性检查；
- [x] 不合格 frame 返回前不修改 `data.costs`，正常数据在本步也仍不修改它；
- [x] `predictive_nav_mppi` 与 `predictive_nav_bringup` 已构建通过；
- [ ] 在运行仿真中看到正常新鲜 `odom` prediction 的 accepted 日志；
- [ ] 停止 prediction_node 后看到 stale rejection 日志；
- [ ] 用测试消息完成错误 frame 和 NaN/Inf 的运行期注入验证。

## 下一步

[09_计算候选轨迹与障碍物距离](../09_计算候选轨迹与障碍物距离/README.md)：只有数据可信后，
才开始遍历 MPPI 候选轨迹与 prediction。
