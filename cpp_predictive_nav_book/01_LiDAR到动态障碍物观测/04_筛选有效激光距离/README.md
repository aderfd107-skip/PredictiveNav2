# 04：筛选有效激光距离

## 这一小步完成了什么

`/scan` 中的每一项 `ranges[i]` 都只是一个原始传感器读数，不能假设它一定能用于数学计算。真实 LiDAR 可能给出：

- `NaN`：不是一个数字；
- `Inf`：无穷远，通常表示这一束激光没有得到有效回波；
- 太近的距离：可能是机器人自身、LiDAR 安装结构或不可靠近场；
- 太远的距离：测量误差更大，而且目前的局部避障不需要处理过远物体。

这一步已经把真实项目节点扩展为：

```text
/scan
  → 遍历每一个 ranges[i]
  → 丢弃非有限值和范围外值
  → 保留 {原始光束下标, 距离} 的有效列表
  → 每 10 帧打印保留和丢弃的数量
```

它仍然不判断“这些点属于哪个障碍物”。下一步才把有效距离转成二维坐标；第 07 步才聚类。

## 本次只修改了一个真实项目文件

```text
src/predictive_nav_perception/src/scan_info_node.cpp
```

你不需要手动写入代码。现在先构建、运行、看输出，再回来看解释。

## 第 1 步：重新构建

在新终端逐行复制：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
```

因为 C++ 文件已经改变，必须重新执行 `colcon build`；否则 `ros2 run` 仍可能运行上一次编译的旧版本。

## 第 2 步：保持 Gazebo 运行并启动节点

让第 01 步启动的 Gazebo 保持运行。若它已停止，在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/aderfd/PredictiveNav2/install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

回到构建节点的终端，运行：

```bash
ros2 run predictive_nav_perception scan_info_node
```

你会看到类似：

```text
Waiting for LaserScan messages on /scan; keeping distances in [0.15, 6.00] m.
scan #10 | frame=lidar_link | ... | ranges=... | kept=... |
discarded(non_finite=..., out_of_range=...) | ...
```

只需要先看懂三个数字：

- `ranges`：LiDAR 这一圈原始距离总数；
- `kept`：通过筛选、可以进入下一步算法的距离数；
- `discarded(...)`：被丢弃的数量和原因。

一般会满足：

```text
ranges = kept + non_finite + out_of_range
```

这就是一个很重要的调试检查：如果不相等，说明过滤逻辑或统计逻辑有 bug。

## 第 3 步：只改参数，不改代码

你不需要编辑 `.cpp` 文件。ROS 2 可以在启动节点时临时传入参数。

按 Ctrl + C 停止节点，然后运行：

```bash
ros2 run predictive_nav_perception scan_info_node --ros-args \
  -p min_detection_range:=0.50 \
  -p max_detection_range:=3.00
```

它的意思是：这次只保留 `0.50 m` 到 `3.00 m` 之间的激光距离。

你应观察到：`kept` 通常会减少，`out_of_range` 通常会增加。这不是程序变坏，而是你要求它使用更窄的有效范围。

重新以默认参数启动只需：

```bash
ros2 run predictive_nav_perception scan_info_node
```

参数只对这一次运行有效，并没有修改源码。

## 新增数据结构：`ValidRange`

代码开头新增：

```cpp
struct ValidRange
{
  std::size_t beam_index{0U};
  float range_m{0.0F};
};
```

这是一个很小的 C++ `struct`，表示“一条保留下来的激光”。它保存两样东西：

- `beam_index`：它原先在 `ranges` 列表中的第几个位置；
- `range_m`：它测得的距离，单位米。

为什么不只保存距离？因为第 05 步还要根据第几个光束，计算它对应的角度：

```text
angle = angle_min + beam_index × angle_increment
```

有角度和距离后，才能计算二维点 `(x, y)`。所以当前保留索引是为下一步准备，而不是多余信息。

## `RangeFilterResult`：不仅保留结果，也保留诊断信息

```cpp
struct RangeFilterResult
{
  std::size_t total_count{0U};
  std::size_t non_finite_count{0U};
  std::size_t out_of_range_count{0U};
  std::vector<ValidRange> valid_ranges;
};
```

它表示“一整帧激光筛选后的结果”。`std::vector<ValidRange>` 是许多 `ValidRange` 组成的可变长度列表。

代码没有悄悄删除坏数据，而是分别记下“为什么删除”。这对真实机器人很重要：若某天 `non_finite_count` 突然异常增大，你知道要检查 LiDAR 数据，而不是误以为聚类算法坏了。

## 两个 ROS 参数

构造函数中有：

```cpp
min_detection_range_ = declare_parameter<double>("min_detection_range", 0.15);
max_detection_range_ = declare_parameter<double>("max_detection_range", 6.00);
```

这两行的意思是：

- 声明节点允许用户设置两个 `double` 小数参数；
- 如果用户不设置，则使用默认值 `0.15 m` 和 `6.00 m`；
- 把最终数值保存到成员变量 `min_detection_range_` 和 `max_detection_range_`。

参数比把数字写死在算法里更好：以后真机 LiDAR 安装高度、场地大小或任务范围变化时，可以改启动参数，而不必每次修改和编译 C++。

## `filter_ranges()`：真正的筛选函数

```cpp
RangeFilterResult filter_ranges(const sensor_msgs::msg::LaserScan & scan) const
```

它的输入是一整条 `LaserScan`，输出是刚才定义的 `RangeFilterResult`。

参数中的 `const ... & scan` 可以先按下面理解：

- `const`：本函数只看原始激光，不会修改它；
- `&`：不复制整条包含大量距离的消息，直接只读地使用它；
- `scan`：这条消息在函数内部的名字。

函数先得到最终有效范围：

```cpp
effective_min = max(传感器最小量程, 用户最小参数)
effective_max = min(传感器最大量程, 用户最大参数)
```

这样不会因为用户设置了不符合传感器能力的数字，就相信传感器量程之外的数据。

接着的循环：

```cpp
for (std::size_t index = 0U; index < scan.ranges.size(); ++index)
```

和 C 的 `for` 循环逻辑相同：从第 0 个距离开始，逐个检查，直到 `ranges` 的最后一个。`++index` 就是 `index = index + 1` 的简洁写法。

每个距离依次经过两关：

1. `std::isfinite(range)`：不是有限数就计入 `non_finite_count` 并 `continue` 跳到下一束；
2. 距离是否落在最终范围内：不在就计入 `out_of_range_count` 并跳过。

只有两关都通过，才会执行：

```cpp
result.valid_ranges.push_back(ValidRange{index, range});
```

`push_back` 是向 vector 末尾增加一个元素。花括号中的 `index, range` 对应 `ValidRange` 的两个字段。

## 回调函数中发生了什么

每收到一条 `/scan`，现在先执行：

```cpp
const RangeFilterResult filter_result = filter_ranges(*message);
```

`*message` 可以理解为“取得共享指针里面那一整条 LaserScan”。然后把筛选结果保存到 `filter_result`。

目前节点只打印 `filter_result` 的统计信息；下一步会遍历 `filter_result.valid_ranges`，把每个“距离 + 光束索引”变成一个二维坐标点。

## 为什么这一小步属于项目，而不是普通 C++ 练习

后续聚类、跟踪、预测都建立在这些点是可信数据的前提上。若把 NaN、Inf 或量程外数直接传给 `cos`、`sin`、距离计算或卡尔曼滤波，就可能产生 NaN 坐标，最终导致整个算法的结果失效。

因此，“先明确过滤无效观测”是机器人感知系统的基本工程习惯。

## 遇到问题怎么办

### 节点输出仍然是旧的 `valid_range=...`

这说明运行的是第 03 步编译出的旧可执行文件。停止节点后，重新执行：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
```

再运行节点。

### `kept` 是 0

先用默认参数运行。若默认仍为 0，执行：

```bash
ros2 topic echo /scan --once
```

确认 `ranges` 是否确实有数据，并把节点第一行和一条 `scan #...` 日志发给我。不要先随意把范围设置得极大。

### `non_finite` 很多

部分 LiDAR 或 Gazebo 场景出现无回波时会给出 `Inf`，这不一定是错误。只要仍有足够的 `kept` 点供后续聚类使用即可。若它突然比之前多很多，才需要检查场景、传感器或桥接。

## 这一小步的完成标准

- [ ] 重新构建成功。
- [ ] 默认运行时日志含有 `kept`、`non_finite` 和 `out_of_range`。
- [ ] 我知道 `ranges` 中并非每个数都可以直接用于算法。
- [ ] 我用启动参数试过一次缩窄有效距离范围，看到统计变化。
- [ ] 我能解释为什么要保留 `beam_index`。

完成后，把默认运行的一行 `scan #...` 日志和缩窄参数运行的一行日志发给我。下一步将把这些有效距离正式变成 LiDAR 坐标系中的二维点 `(x, y)`。
