# 06：把 LiDAR 点统一到 `odom` 坐标系

## 这一小步完成了什么

第 05 步把每一束有效激光转换成了 `lidar_link` 中的二维点 `(x, y)`。这种坐标表示“点相对 LiDAR 在哪里”。机器人一移动，即使是静止的墙，墙在 `lidar_link` 中的位置也会改变。

后续多目标跟踪要区分“机器人自己动了”和“障碍物真的动了”，因此必须把点统一到短时间连续、稳定的 `odom` 坐标系。

本步新增的数据流是：

```text
有效距离 → lidar_link 中的二维点
         → 查询 odom ← lidar_link 的 TF
         → odom 中的二维点（TrackingPoint）
```

这里的箭头 `odom ← lidar_link` 读作：**把原本在 `lidar_link` 中描述的数据，转换为 `odom` 中描述的数据。**

## 为什么不能直接在 `lidar_link` 中跟踪

想象机器人向前走 1 米，旁边一张静止桌子没有动：

```text
在 lidar_link 中：桌子看起来向后移动约 1 米
在 odom 中：      桌子仍大致留在原位置
```

如果直接在 `lidar_link` 中计算速度，系统会把所有静止墙壁、家具都误认为在移动。把点转到 `odom` 后，后续卡尔曼滤波估计的才是障碍物相对地面的短期运动。

本项目暂时不用 `map` 做在线跟踪：AMCL 会修正 `map → odom`，这种修正可能让 `map` 中的坐标瞬间跳动，进而制造假的障碍物速度。`odom` 虽会慢慢漂移，但在短期局部跟踪中连续平滑，更适合本任务。

## 当前 TF 链是什么

当前仿真中最重要的关系可以先看作：

```text
map → odom → base_footprint → lidar_link
```

- `map → odom`：AMCL 负责，用于全局定位；
- `odom → base_footprint`：机器人本体的短期移动；
- `base_footprint → lidar_link`：LiDAR 安装在机器人上的固定位置。

第 06 步要查询的是从 `lidar_link` 到 `odom` 的完整组合变换。TF 会自动沿链查找，你不用手动先算两个变换再相乘。

## 本次新增的 ROS 2 依赖

感知包新增了：

```text
tf2              TF 的底层变换和异常类型
tf2_ros          TF Buffer、TransformListener
tf2_geometry_msgs 把 PointStamped 按 TransformStamped 变换的函数
geometry_msgs    PointStamped、TransformStamped 等消息类型
```

`package.xml` 说明运行依赖，`CMakeLists.txt` 说明构建时怎样链接它们。你现在不用手写这些配置；它们已在真实包中更新。

## 先构建并运行

打开一个新终端，逐行复制：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
```

另一个终端保持第 01 步的 Gazebo / Nav2 场景运行。没有运行时，TF 和 `/scan` 都不会存在。

然后运行：

```bash
ros2 run predictive_nav_perception scan_info_node
```

成功时会同时看到两类点：

```text
first valid point       ... lidar_link=(x, y) m
first transformed point ... odom=(x, y) m
scan #... | lidar_points=... | tracking_points=... | tf_failures_total=... | ...
```

`lidar_points` 与 `tracking_points` 数量通常应相同。`tf_failures_total` 是节点从本次启动开始累计的 TF 失败次数。

刚启动的最初几帧可能在 TF Buffer 尚未收到完整链路时失败，因此它可能先从 `0` 变成 `1`、`2` 或 `3`；这不一定是错误。关键是：当后续日志中的 `tracking_points` 持续大于 0，且 `tf_failures_total` **不再增加**，说明 TF 已稳定可用。

## 程序如何收到 TF

构造函数中新加入两个成员：

```cpp
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;
```

`TransformListener` 在后台订阅 ROS 2 的 `/tf` 和 `/tf_static`；`Buffer` 将收到的坐标关系按时间暂存起来，供我们之后查询。

构造函数初始化：

```cpp
tf_buffer_(this->get_clock()),
tf_listener_(tf_buffer_, this, false)
```

先不用背语法。它的含义是：创建一个按节点时钟管理的 TF 缓存，并让监听器把本节点收到的 TF 写入该缓存。最后的 `false` 表示不额外启动一条 spin 线程；现有 `rclcpp::spin()` 已经负责处理本节点的消息。

## 关键原则：查询 scan 原始时间戳，而不是“最新 TF”

本项目代码查询：

```cpp
tf_buffer_.lookupTransform(
  tracking_frame_,
  scan.header.frame_id,
  rclcpp::Time(scan.header.stamp));
```

四个参数依次是：

1. 目标 frame：默认 `odom`；
2. 源 frame：该帧 scan 自己声明的 `lidar_link`；
3. 时间：`scan.header.stamp`，即这圈激光实际测到的时刻；

这里故意**不等待** TF。感知节点与 `TransformListener` 使用同一个 ROS 2 执行线程；在回调中等待 TF 会阻塞这个线程本身接收新的 TF。若当前时间点尚未进入 Buffer，代码会捕获异常并安全跳过本帧，等下一帧再试。

为什么不使用“现在最新的 TF”？如果机器人正在移动，当前机器人姿态可能已经比本帧激光晚几十毫秒。把旧激光拿去套新姿态，会让同一面墙的点产生人为错位；这会直接损害后续聚类和速度估计。

## 一次查询，转换整帧所有点

`transform_to_tracking_frame()` 先为整帧查询一次 TF，再在循环中把每个点调用 `tf2::doTransform()`。

这样做比“每个点单独查询一次 TF”更正确也更高效：一帧 LiDAR 的所有点共享同一时刻、同一坐标变换。如果对 500 个点各查询一次，既浪费时间，也可能在查询之间得到不同 TF。

代码使用 `geometry_msgs::msg::PointStamped`，因为普通 `(x, y)` 数字没有说明“它属于哪个坐标系、哪个时刻”。`PointStamped` 带有：

```text
header.frame_id = lidar_link
header.stamp    = 本帧 scan 时间
point.x/y/z     = 当前激光点坐标
```

`tf2::doTransform()` 会处理完整的平移和旋转；即使 LiDAR 不是装在机器人中心、安装姿态有旋转，也不需要我们手写三角函数。

## `TrackingPoint` 是什么

```cpp
struct TrackingPoint
{
  std::size_t beam_index{0U};
  double range_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
};
```

它表示同一束激光最终在跟踪坐标系中的位置。默认跟踪坐标系是 `odom`，但代码把它写成参数 `tracking_frame`，便于以后在其他受控实验中更换 frame；本项目在线跟踪仍固定推荐 `odom`。

## TF 失败时为什么丢弃这一帧

如果 TF 查不到，代码捕获 `tf2::TransformException`，增加 `tf_failure_count_` 并丢弃当前 scan。

绝不能偷偷套用上一帧的变换：那会把旧姿态用到新激光上，制造错误世界坐标。宁可少用一帧，也不能向跟踪器输入空间位置错误的数据。

## 常见问题与检查顺序

### `tf_failures` 不为 0，或不断显示 `Skipping scan...`

先在另一个已 `source` 环境的终端运行：

```bash
ros2 run tf2_ros tf2_echo odom lidar_link
```

成功时会持续显示坐标变换。若命令报“frame does not exist”或持续等待：

1. 确认 Gazebo / Nav2 launch 仍在运行；
2. 执行 `ros2 topic echo /tf --once`，确认 TF 在发布；
3. 执行 `ros2 topic echo /scan --once`，确认 `frame_id` 确实是 `lidar_link`；
4. 把节点中完整的 `Skipping scan...` 日志发给我。

不要为了消除报错，把查询时间改为“最新时间”；先确认 TF 链与时间戳。

### 第一个 `lidar_link` 点和 `odom` 点看起来不同

这是正常现象。两者的原点不同：`lidar_link` 原点在机器人激光雷达上，`odom` 原点是机器人启动后的局部参考。你关心的是：当机器人走动时，静止物体在 `odom` 中大致保持稳定。

### 一开始偶尔有一两次 TF 失败

刚启动时 TF Buffer 还没收到足够历史数据，偶尔失败可以接受。若持续失败，则按上一项检查。

## 这一小步的完成标准

- [ ] 重新构建成功。
- [ ] 日志中同时出现 `lidar_link=(...)` 与 `odom=(...)`。
- [ ] `tracking_points` 大于 0，且通常等于 `lidar_points`。
- [ ] 稳定运行后 `tracking_points` 持续大于 0，且 `tf_failures_total` 不再增加（启动初期允许有少量累计失败）。
- [ ] 我能解释为什么多目标跟踪使用 `odom`，以及为什么 TF 必须查询 scan 的时间戳。

第 07 步会在这些 `odom` 点上做欧氏聚类，把一堆相邻点变成一个个障碍物观测。

## 本章关联的 ROS 2 知识

**本章必须懂**：TF 不只是“坐标变换公式”，而是“某个时刻，一个 frame 到另一个 frame 的关系”。因此本章必须使用 `scan.header.stamp` 查询 `odom ← lidar_link`，不能随手拿最新 TF。你已经通过 `tracking_points` 和不再增长的 `tf_failures_total` 验证了它可用。

**可选扩展**：现在不要求理解 TF Buffer 的内部缓存机制、`map → odom` 的 AMCL 数学细节，或多线程 executor。它们不阻塞你进入聚类和跟踪。
