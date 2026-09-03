# 05：确定预测时间轴

> 本步目标：在真正预测位置之前，先固定“预测从哪个未来时刻开始、看多远、相邻未来点相隔多久”。本步仍然不计算 CV 位置，也不发布 prediction 消息。

## 1. 什么是预测时间轴？

假设 tracking 在测量时间 `100.0 s` 发布一条 Track。预测模块不能只说“以后的位置”，而要明确每个未来位置对应的时刻：

```text
Track 的当前测量时间（header.stamp） = 100.0 s

offset = 0.2 s  ->  预测 100.2 s 的位置
offset = 0.4 s  ->  预测 100.4 s 的位置
...
offset = 2.0 s  ->  预测 102.0 s 的位置
```

这串 `[0.2, 0.4, ..., 2.0] s` 就是**预测时间轴**。它不是“循环十次”这么简单，而是未来点的时间含义。第 04 模块做风险评估时，才能比较：

```text
机器人在 0.6 s 后的候选位置
              vs
障碍物在 0.6 s 后的预测位置
```

只有双方比较的是同一时刻，距离和碰撞风险才有意义。

## 2. 本项目的默认选择

本步新增两个 ROS 参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `prediction_horizon_s` | `2.0` | 从当前 Track 向未来最多看 2.0 秒 |
| `prediction_dt_s` | `0.2` | 相邻预测点相隔 0.2 秒 |

默认时间轴为：

```text
[0.20, 0.40, 0.60, 0.80, 1.00,
 1.20, 1.40, 1.60, 1.80, 2.00] s
```

也就是 10 个未来点。

### 为什么选 2.0 秒？

你的动态障碍物预测目前采用 CV（匀速）模型。它适合短时间近似，预测太远会把“目标可能转弯、减速或遮挡”的误差放大。2 秒是室内低速小车的合理初始值，不代表永远正确；之后会用仿真和 rosbag 评估再调。

### 为什么每 0.2 秒一个点？

当前 `/scan` 和 Track 通常约 10 Hz，即每 0.1 秒有一帧观测。预测点使用 0.2 秒间隔，相当于每秒输出 5 个未来点：足以表达风险变化，又不会让下游 MPPI 接收过密、冗余的数据。

注意：`prediction_dt_s` 不等于 LiDAR 帧间隔，也不等于 tracking 卡尔曼滤波的真实 `dt`。它只是“向未来采样的间隔”。

## 3. 两个必须分清的时间

```text
传感器/Track 时间：这一帧状态是什么时候测到的？
预测 offset：从这个测量时刻再向未来推多久？
```

未来预测的绝对时间是：

```text
prediction_time = track_header.stamp + time_offset
```

绝不能用“回调函数在电脑上运行的 `now()`”替代 `track_header.stamp`。否则 rosbag 回放、传输延迟或仿真时间下，预测会与真实测量时刻错位。

## 4. 为什么时间轴不包含 `0.0 s`？

`0.0 s` 的状态就是输入 Track 当前的位置和速度，第二模块已经发布过了。第三模块输出的是“**未来**预测”，因此默认从 `0.2 s` 开始。

```text
0.0 s：Track 已知当前状态，不重复发布
0.2 s：第一项真正的预测
...
2.0 s：预测范围的最后一项
```

## 5. horizon 不能整除 dt 时怎么办？

例如你设置：

```text
prediction_horizon_s = 1.0
prediction_dt_s = 0.3
```

时间轴会是：

```text
[0.3, 0.6, 0.9, 1.0] s
```

不会出现 `1.2 s`，因为它已经超出 horizon；也不会漏掉 `1.0 s`，因为最后会补上精确终点。这样“预测未来 1 秒”这句配置才是严格成立的。

如果 `dt > horizon`，例如 horizon 为 1 秒、dt 为 2 秒，仍会得到 `[1.0] s`：至少保留一个“预测到 horizon 终点”的结果。

## 6. 代码做了什么？

### 参数声明

```cpp
prediction_horizon_s_ = declare_parameter<double>("prediction_horizon_s", 2.0);
prediction_dt_s_ = declare_parameter<double>("prediction_dt_s", 0.2);
```

`declare_parameter<double>` 的含义是：

- 若运行命令没有传参数，使用默认值；
- 若运行命令传入参数，使用你传入的值；
- 值保存到成员变量里，后续所有预测共用同一套时间轴。

### 构建时间轴

`make_prediction_time_offsets()` 返回一个 `std::vector<double>`：

```cpp
std::vector<double> prediction_time_offsets_s_;
```

`vector` 可以理解为 C 语言中“长度可变的 `double` 数组”，但它会自动管理内存，不需要你手动 `malloc` / `free`。

函数会检查：

- horizon 必须是有限且大于 0 的数；
- dt 必须是有限且大于 0 的数；
- 时间轴最多 100 个点，防止把 dt 误写得极小而让下游负担失控。

配置错误时节点会明确拒绝启动，而不是偷偷产生无穷循环或错误预测。

## 7. 构建和运行

先构建：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction
source install/setup.bash
```

使用默认时间轴运行：

```bash
ros2 run predictive_nav_prediction prediction_node
```

刚启动时应首先看到：

```text
prediction time axis | horizon=2.00 s | dt=0.20 s | point_count=10 |
offsets=[0.20, 0.40, ..., 2.00] s
```

你也可以临时传入参数，不必修改代码：

```bash
ros2 run predictive_nav_prediction prediction_node --ros-args \
  -p prediction_horizon_s:=1.0 \
  -p prediction_dt_s:=0.3
```

预期看到：

```text
point_count=4 | offsets=[0.30, 0.60, 0.90, 1.00] s
```

### 故意传错参数，看看防御性检查

下面命令会让节点拒绝启动，这是正确行为：

```bash
ros2 run predictive_nav_prediction prediction_node --ros-args \
  -p prediction_horizon_s:=0.0
```

不要在 tracking、Nav2 正运行时频繁测试错误配置；单独开一个 prediction 终端测试后，按 `Ctrl+C` 关闭即可。

## 8. 本步完成标准

- [ ] 默认启动日志显示 2.0 秒 horizon、0.2 秒 dt、10 个 offset；
- [ ] 能解释 horizon 是总预测范围，dt 是未来点间隔；
- [ ] 明白 offset 相对输入 Track 的 `header.stamp`，不是相对电脑当前时间；
- [ ] 用 `1.0 s / 0.3 s` 参数验证得到 `[0.30, 0.60, 0.90, 1.00] s`；
- [ ] 知道此时还没有计算或发布未来位置。

## 本章必须懂的 ROS 2 / C++ 知识

- ROS 参数让同一份代码适配不同预测范围，而无需反复改代码、重新编译。
- `std::vector<double>` 是会自动管理内存的动态数组。
- `header.stamp + offset` 才是未来预测时刻；不要混淆传感器时间、回调时间和预测时间。
- 对参数做范围检查是工程代码的一部分，能避免错误配置在机器人运行中变成隐患。

## 下一步

[06_CV传播未来位置](../06_CV传播未来位置/README.md)：时间轴已固定，下一步才使用 CV 模型，把 Track 的位置和速度传播到每个 offset 对应的未来位置。
