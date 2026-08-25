# 05：把有效激光距离变成二维点 `(x, y)`

## 这一小步完成了什么

第 04 步已经从一整圈原始 `ranges` 中留下了可信数据：

```text
{beam_index, range_m}
```

但“第 120 束激光测到 2.3 米”仍然不是地图中的点。它必须结合这束激光的角度，才能表示为 LiDAR 坐标系中的二维位置：

```text
{beam_index, range_m}
  → angle_rad
  → {x_m, y_m}
```

完成本步后，`scan_info_node` 会在内部把每个有效距离转换成一个二维点；每十帧打印一个点的示例和本帧点数，证明计算实际发生了。

它仍然**不会**：

- 判断点属于墙、家具还是动态 actor；
- 聚类；
- 发布障碍物 topic；
- 把点转换到 `odom`；
- 订阅 Gazebo 真值 Marker。

这些点目前在 `lidar_link` 坐标系中。第 06 步才会用 TF 把它们统一到 `odom`。

## 先建立正确的画面感

把 LiDAR 想成站在机器人上的一个小圆盘：每次扫描都向不同方向发射一束光。

```text
                    y（左）
                    ↑
                    |
                    |   • (x, y)：某束激光的终点
                    |  /
                    | /  r：距离 range_m
                    |/ θ
  ←─────────────────L─────────────────→ x（前）
                   LiDAR
```

ROS 中机器人常用的二维约定是：

- `x` 正方向：前方；
- `y` 正方向：左方；
- 绕 `z` 轴逆时针为正角度。

因此：

- 正前方的点应接近 `(正数, 0)`；
- 左侧的点应接近 `(0, 正数)`；
- 右侧的点应接近 `(0, 负数)`。

这只是 `lidar_link` 中的相对位置。机器人一移动，同一面墙在该坐标系中的点也会随之改变；这正是第 06 步需要 TF 的原因。

## 一条 LaserScan 到底怎样确定角度

`LaserScan` 有三个本步需要的字段：

```text
angle_min        第一束激光的角度，单位 rad
angle_increment  相邻两束激光的角度差，单位 rad
ranges[i]        第 i 束激光的距离，单位 m
```

对原始下标为 `i` 的激光束：

```text
angle_i = angle_min + i × angle_increment
```

第 04 步保留 `beam_index` 的唯一原因就在这里：有效数据在过滤后的位置不再等于原始光束编号。

例如，一帧激光中第 3 束是 NaN 被丢弃；后面原本第 4 束的有效点，仍必须使用 `beam_index = 4`，绝对不能把它当作“有效列表的第 3 个元素”。否则角度会整体错位。

## 极坐标转换公式

这是本步唯一的新数学：

\[
x = r\cos(\theta)
\]

\[
y = r\sin(\theta)
\]

其中：

- \(r\)：`range_m`，单位米；
- \(\theta\)：这束激光的角度，单位弧度；
- \(x,y\)：点在 `lidar_link` 中的坐标，单位米。

### 用三个最简单的例子检查公式

| 距离 `r` | 角度 `θ` | 预期 `(x, y)` | 含义 |
| --- | --- | --- | --- |
| `2.0` | `0` | `(2.0, 0.0)` | 正前方 2 米。 |
| `3.0` | `π/2` | `(0.0, 3.0)` | 左侧 3 米。 |
| `1.5` | `-π/2` | `(0.0, -1.5)` | 右侧 1.5 米。 |

`std::sin()` 和 `std::cos()` 的参数必须是**弧度**，不是角度制。例如 `90` 不是 90 度，而是 90 弧度；90 度应写为 `π / 2`，约等于 `1.5708`。

## 这一步新增的数据结构

在 `src/predictive_nav_perception/src/scan_info_node.cpp` 中，紧接第 04 步的 `ValidRange` 后面，自己先尝试定义：

```cpp
struct CartesianPoint
{
  std::size_t beam_index{0U};
  double range_m{0.0};
  double angle_rad{0.0};
  double x_m{0.0};
  double y_m{0.0};
};
```

为什么还要保留 `beam_index`、`range_m` 和 `angle_rad`，而不是只保存 `x_m/y_m`？

- 方便日志和单元测试：你能知道这个点由哪一束、哪个距离算来；
- 方便排错：若点位置不对，可以区分是过滤错、角度错还是三角函数错；
- 后续可视化/诊断仍可能需要这些原始信息。

这里使用 `double` 做三角函数和坐标计算。`LaserScan` 本身用 `float` 节省传输空间，但几何计算用 `double` 是常见且安全的选择。

## 先做一个不依赖 ROS 的手算练习

在脑中或临时 `.cpp` 文件里完成这段小练习。它不需要改项目：

```cpp
#include <cmath>
#include <iostream>

int main()
{
  const double range_m = 2.0;
  const double angle_rad = 0.0;

  const double x_m = range_m * std::cos(angle_rad);
  const double y_m = range_m * std::sin(angle_rad);

  std::cout << "(" << x_m << ", " << y_m << ")\n";
  return 0;
}
```

你应看到接近：

```text
(2, 0)
```

将 `angle_rad` 改为 `1.5707963267948966` 再运行，思考为什么输出的 x 不一定是机器显示的严格 `0`，而是非常接近 0 的小数。这是浮点数近似，不是公式错误。

## 在真实节点中新增转换函数

不要把坐标计算塞进 `scan_callback()` 的深处。像第 04 步把筛选提取为 `filter_ranges()` 一样，本步也写一个职责单一的成员函数：

```cpp
std::vector<CartesianPoint> ranges_to_lidar_points(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<ValidRange> & valid_ranges) const
```

这个函数的输入是：

- 一整条 scan：提供 `angle_min` 与 `angle_increment`；
- 已筛选的 `valid_ranges`：只提供可信距离和原始 `beam_index`。

输出是一组已经在 `lidar_link` 坐标系中的点。

### 先写伪代码

在写 C++ 前，确保你能看懂下面每行：

```text
创建一个空的 points 列表
预留 valid_ranges.size() 个位置

遍历每个 valid_range：
  angle = scan.angle_min + valid_range.beam_index × scan.angle_increment
  x = valid_range.range_m × cos(angle)
  y = valid_range.range_m × sin(angle)
  将 beam_index、range、angle、x、y 放进 points

返回 points
```

### 再自己实现

提示：需要使用的工具已经在原文件中出现过或很简单：

- `std::vector<CartesianPoint> points;`
- `points.reserve(valid_ranges.size());`
- 范围 for：`for (const ValidRange & valid_range : valid_ranges)`；
- `static_cast<double>(...)`；
- `std::cos()`、`std::sin()`；
- `points.push_back(CartesianPoint{...});`

写完、构建通过后再看下面的参考实现。

```cpp
std::vector<CartesianPoint> ranges_to_lidar_points(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<ValidRange> & valid_ranges) const
{
  std::vector<CartesianPoint> points;
  points.reserve(valid_ranges.size());

  for (const ValidRange & valid_range : valid_ranges) {
    const double angle_rad = static_cast<double>(scan.angle_min) +
      static_cast<double>(valid_range.beam_index) *
      static_cast<double>(scan.angle_increment);
    const double range_m = static_cast<double>(valid_range.range_m);

    points.push_back(CartesianPoint{
      valid_range.beam_index,
      range_m,
      angle_rad,
      range_m * std::cos(angle_rad),
      range_m * std::sin(angle_rad)});
  }

  return points;
}
```

### 逐句解释

```cpp
const std::vector<ValidRange> & valid_ranges
```

这表示“只读地使用已有的有效距离列表，不复制它”。`const` 防止你在转换函数中意外删除或修改第 04 步的结果；`&` 避免复制所有数据。

```cpp
points.reserve(valid_ranges.size());
```

这不是添加空点。它只是提前告诉 `vector`：“我最多大约会放这么多个点。”之后 `push_back()` 仍然逐个添加真正的点。这样可减少内存反复扩容，是一种简单的性能习惯。

```cpp
static_cast<double>(valid_range.beam_index)
```

`beam_index` 是 `std::size_t`（无符号整数）；角度计算需要小数。`static_cast<double>` 明确告诉 C++ 我们有意把它用于浮点运算。相比隐式转换，它更容易阅读和检查。

```cpp
for (const ValidRange & valid_range : valid_ranges)
```

这是范围 for 循环，意思是“依次遍历列表里每个元素”。它与 C 中按下标写循环功能相同，但这里不需要自己处理 `size()` 与下标；因为不修改元素，所以使用 `const ... &`。

## 把转换结果接入回调函数

在 `scan_callback()` 中，筛选后新增：

```cpp
const std::vector<CartesianPoint> lidar_points =
  ranges_to_lidar_points(*message, filter_result.valid_ranges);
```

此行必须放在：

```cpp
const RangeFilterResult filter_result = filter_ranges(*message);
```

之后，因为它依赖 `filter_result.valid_ranges`。

日志不要打印全部几百个点。只在当前原有“每十帧打印一次”的分支中加一个示例：

```cpp
if (!lidar_points.empty()) {
  const CartesianPoint & first = lidar_points.front();
  RCLCPP_INFO(
    get_logger(),
    "first valid point | beam=%zu | range=%.3f m | angle=%.4f rad | "
    "lidar_link=(%.3f, %.3f) m",
    first.beam_index, first.range_m, first.angle_rad, first.x_m, first.y_m);
}
```

`front()` 得到 vector 的第一个元素；在调用前必须先检查 `empty()`。如果没有有效距离却访问 `front()`，程序会产生未定义行为，可能崩溃。

还可以在现有摘要中新增：

```text
points_lidar_link=...
```

它在数值上应等于 `kept`，因为每个有效距离恰好转换为一个二维点。若两者不相等，优先检查你是否意外跳过了某个 valid range。

## 本次不需要修改的文件

只要代码仍然只使用 `rclcpp`、`sensor_msgs` 和 C++ 标准库，本步不新增 ROS package 依赖。因此不必修改：

```text
src/predictive_nav_perception/CMakeLists.txt
src/predictive_nav_perception/package.xml
```

它们已经使用 C++17，并已引入 `sensor_msgs`。`<cmath>` 和 `<vector>` 都属于 C++ 标准库，不需要在 ROS 的 `package.xml` 声明。

## 构建和运行

### 终端 A：启动已有仿真

若 Gazebo 未运行：

```bash
cd /home/aderfd/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

### 终端 B：构建并运行你的修改

```bash
cd /home/aderfd/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node
```

预期：每十帧会先出现原有筛选摘要，再出现或包含一条类似：

```text
first valid point | beam=... | range=... m | angle=... rad |
lidar_link=(..., ...) m
```

数值不需要和示例完全相同。判断正确性的方法是：

- `points_lidar_link`（若你打印它）等于 `kept`；
- `range_m` 与第 04 步留下的有效范围一致；
- 角度满足 `angle_min + beam_index × angle_increment`；
- `(x, y)` 都是有限数；
- 节点持续运行而不崩溃。

## 可选的额外验证：用一个固定 scan 检查函数

现在 `ranges_to_lidar_points()` 仍是 node 的私有成员函数，不便直接写 gtest。首版可以先使用运行日志验证；第 07 步重构聚类算法时，会把几何转换提取为独立 `.hpp/.cpp`，再为它写单元测试。

如果你想提前练习测试，可以把转换逻辑复制到一个普通小函数，并设计这三组输入：

| `angle_min` | `angle_increment` | `beam_index` | `range_m` | 预期 |
| --- | --- | --- | --- | --- |
| `0` | `π/2` | `0` | `2` | `(2, 0)` |
| `0` | `π/2` | `1` | `3` | `(0, 3)` |
| `-π/2` | `π/2` | `0` | `1.5` | `(0, -1.5)` |

用 `EXPECT_NEAR` 比较浮点数，不使用 `EXPECT_EQ`：

```cpp
EXPECT_NEAR(actual_x, expected_x, 1e-9);
```

## 常见错误与排查

### 错误 1：用有效列表的位置代替 `beam_index`

错误想法：

```cpp
for (std::size_t i = 0; i < valid_ranges.size(); ++i) {
  const double angle = scan.angle_min + i * scan.angle_increment;
}
```

一旦中间有一束无效激光被过滤，`i` 就不再是原始光束下标，后续所有点角度都会偏移。

正确做法永远是：

```cpp
valid_range.beam_index
```

### 错误 2：把度数直接传给 `sin/cos`

错误：`std::cos(90.0)` 不代表 90 度。

正确：LaserScan 自带的 `angle_min`、`angle_increment` 已经是弧度，直接使用即可；不要再自行做角度制转换。

### 错误 3：x/y 前后或左右颠倒

正确公式固定为：

```text
x = r × cos(angle)
y = r × sin(angle)
```

不是把 sin/cos 交换，也不应随意给 y 加负号。若 RViz 以后看起来方向不对，先打印 `angle_min`、检查 `frame_id` 与 TF，再怀疑公式。

### 错误 4：没有重新构建

源码修改后只执行 `ros2 run`，会运行上次 build 的旧二进制。每次改 `.cpp` 都执行：

```bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
```

### 错误 5：以为点已经在 `odom`

本步的 `x_m/y_m` 是 `lidar_link` 中的坐标。日志中必须明确写 `lidar_link=(x, y)`，不要标成地图坐标或 odom。第 06 步会处理这个问题。

## 这一小步的完成标准

- [ ] 我能写出 `angle = angle_min + beam_index × angle_increment`。
- [ ] 我能写出 `x = range × cos(angle)`、`y = range × sin(angle)`。
- [ ] `CartesianPoint` 保存了 beam、距离、角度和 `(x,y)`。
- [ ] 节点只将第 04 步的 `valid_ranges` 转换为二维点。
- [ ] 每十帧看到至少一个 `lidar_link=(x, y)` 示例，且数值有限。
- [ ] 我能解释：这些点仍跟着机器人运动，尚不能直接用于全局/跨帧跟踪。

完成后，把一条包含 `first valid point` 的日志发给我。下一步会学习 TF：在**这条 scan 的原始时间**查询 `odom ← lidar_link`，把本步的点放进连续、可跟踪的坐标系。

## 本章关联的 ROS 2 知识

**本章必须懂**：本章计算出的 `(x, y)` 默认属于消息 header 中的 `lidar_link`，不是世界坐标。数字本身没有 frame 意义；只有把它和 scan 的 frame、时间戳一起看，后续才能正确转换。

**可选扩展**：不必现在上手 PCL、PointCloud2 或三维旋转。这个项目当前是二维 LiDAR，先把二维几何和 frame 关系弄清即可。
