# 03：订阅 `/scan` 并打印基本信息

## 这一小步完成了什么

这一步已经在项目真实源码中加入了第一个 C++ 节点：

```text
src/predictive_nav_perception/src/scan_info_node.cpp
```

它订阅上一节观察过的 `/scan`。每收到 10 条 `LaserScan` 消息，它打印一次摘要：数据来自哪个坐标系、这一圈包含多少距离、角度间隔、传感器有效量程和消息时间。

它**不会**控制机器人、不会发布速度、不会读取 Gazebo 真值，也不会做聚类。它的唯一职责是证明：`predictive_nav_perception` 的 C++ 节点确实收到了真实 LiDAR 数据。

## 本次实际修改的文件

```text
src/predictive_nav_perception/
├── package.xml                     # 新增 rclcpp、sensor_msgs 依赖
├── CMakeLists.txt                  # 登记怎样编译 scan_info_node
└── src/scan_info_node.cpp          # 新的 C++ ROS 2 节点
```

代码已经写好。你现在不需要从空白输入它；先运行它、看输出、再逐段理解。

## 第 1 步：构建新增的 C++ 节点

打开一个新终端。不要使用正在运行 Gazebo 的终端，逐行复制：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
source install/setup.bash
```

最后一行很重要：它让当前终端认识刚刚构建好的 `scan_info_node`。每次重新打开新终端或重新构建后，需要再次执行 `source install/setup.bash`。

构建成功时应有：

```text
Summary: 1 package finished
```

## 第 2 步：保持第 01 步的仿真运行

你需要让第 01 步启动的 Gazebo 保持运行。如果之前已经按 Ctrl + C 停掉了，请在另一个终端重新启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/aderfd/PredictiveNav2/install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

等待 Gazebo 打开，并确认 `/scan` 已经再次出现。此时不要在 Gazebo/RViz 内点击任何内容；这个小节点只读取激光数据。

## 第 3 步：运行你的第一个项目 C++ 节点

回到第 1 步构建的终端，复制：

```bash
ros2 run predictive_nav_perception scan_info_node
```

它先显示：

```text
Waiting for LaserScan messages on /scan...
```

之后会每隔约十条消息显示一行。内容大致如下；具体数字可能不同：

```text
scan #10 | frame=lidar_link | stamp=... | ranges=... |
angle_min=... rad | angle_increment=... rad | valid_range=[..., ...] m
```

看到持续增加的 `scan #10`、`scan #20`、`scan #30`，就证明 C++ 节点持续收到了真实激光消息。

按 **Ctrl + C** 可以停止这个节点；它只会停止当前 `scan_info_node`，不会关闭 Gazebo。想再次运行时，重复第 3 步即可。

## 先用一句话理解代码

这份代码可以先理解为：

```text
创建一个名为 scan_info_node 的 ROS 2 节点
→ 订阅 /scan
→ 每次收到 LaserScan，执行 scan_callback
→ 计数加一
→ 每十次打印一次摘要
```

不要先被长类型名吓到。下面按这个运行顺序解释。

## 第 1～5 行：引入已经存在的工具

```cpp
#include <cstddef>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
```

前两行是 C++ 标准库：`cstddef` 提供计数用的 `std::size_t`，`memory` 提供后面自动管理对象用的 `std::make_shared`。

后两行是 ROS 2 已写好的功能：

- `rclcpp`：写 ROS 2 C++ 节点需要的库；
- `LaserScan`：`/scan` 消息的数据类型定义。

它们类似你在 C 中写 `#include <stdio.h>` 后再用 `printf`：不是我们重写打印或 ROS 通信，而是引入别人已经实现好的功能。

## `class ScanInfoNode : public rclcpp::Node`

```cpp
class ScanInfoNode : public rclcpp::Node
```

这句先不用试图完全背下。它表示：我们定义一个叫 `ScanInfoNode` 的新类型，它是一种 ROS 2 节点，并且拥有普通 ROS 节点的能力，例如订阅话题、记录日志。

`public` 在这里不是“公开变量”，而是 C++ 的继承写法。你现在只要记住：`ScanInfoNode` 可以使用 `rclcpp::Node` 提供的 `create_subscription()` 和 `get_logger()`，就是因为这一句。

## 构造函数：节点刚创建时执行一次

```cpp
ScanInfoNode()
: Node("scan_info_node")
```

`ScanInfoNode()` 与类同名，所以它是构造函数；节点一创建就运行一次。

`: Node("scan_info_node")` 的意思是“先创建父类那部分，并把 ROS 节点名设为 `scan_info_node`”。因此运行 `ros2 node list` 时会看到 `/scan_info_node`。

## 创建订阅：连接到 `/scan`

```cpp
scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
  "/scan",
  rclcpp::SensorDataQoS(),
  [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
    scan_callback(message);
  });
```

现在按参数逐项看：

1. `<sensor_msgs::msg::LaserScan>`：声明这个订阅只接收 LaserScan 类型的消息；
2. `"/scan"`：订阅的话题名，正是第 01 步观察到的名字；
3. `rclcpp::SensorDataQoS()`：传感器常用的通信规则。它优先使用最新数据，偶尔丢一条旧扫描也比排队处理很久以前的扫描更好；
4. 最后的 `[this](...) { ... }`：收到新消息时要做什么。它调用本类的 `scan_callback(message)`。

最后这一段叫 **lambda（匿名函数）**。它只是“把一段很短的处理动作直接写在这里”；目前不要求你自己写 lambda。你只要知道它把新激光消息交给下面的 `scan_callback`。

`ConstSharedPtr` 可以暂时读作“只读的消息”。这个节点只查看激光，不会修改原始 `/scan` 数据。

## 回调函数：每来一条激光就执行一次

```cpp
void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
```

这是普通成员函数。它的参数 `message` 就是新到达的一整条 LaserScan；和第 01 步 `ros2 topic echo /scan --once` 显示的内容是同一类数据。

第一行：

```cpp
++message_count_;
```

表示消息计数加一。`message_count_` 最初是 0；它是类中保存的成员变量，因此每次回调后计数不会忘掉前一次的值。

接着：

```cpp
if (message_count_ % 10U != 0U) {
  return;
}
```

`%` 是求余数。只有计数能被 10 整除时，余数才是 0。因此第 1～9 条消息直接 `return`，第 10、20、30 条才打印。这样终端不会被每秒约 10 行日志淹没。

## 打印的字段来自哪里

`RCLCPP_INFO(...)` 相当于 ROS 2 版本的“打印正常信息”。它读取：

- `message->header.frame_id`：数据最初所在坐标系，预期是 `lidar_link`；
- `message->header.stamp`：这次扫描的时间；
- `message->ranges.size()`：这一圈有多少个距离数字；
- `angle_min` 和 `angle_increment`：每个距离对应方向的计算依据；
- `range_min` 和 `range_max`：传感器声称可信的距离范围。

`message->` 和你在 C 中通过指针访问成员的 `pointer->field` 很像。原因是 ROS 2 用“共享指针”管理消息；此刻你只要把它读作“从这条消息里取字段”。

## 最后两项成员变量

```cpp
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
std::size_t message_count_{0U};
```

第一行保存订阅本身。它必须作为节点对象的一部分长期存在；如果只在构造函数中临时创建、随后消失，订阅就会自动取消，永远收不到消息。

第二行保存已经收到的消息数量。`{0U}` 表示从无符号整数 0 开始。

## `main()`：让节点开始运行

```cpp
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<predictive_nav::ScanInfoNode>());
rclcpp::shutdown();
```

这三句按顺序表示：启动 ROS 2、创建节点并持续等待/处理消息、在程序结束时清理 ROS 2 资源。`spin` 会一直运行，所以只有你按 Ctrl + C 后才会继续到 `shutdown()`。

## 这一步新增了哪些 C++ 知识

你现在不需要默写，但已经在真实项目中见到了：

- `class`：把节点状态和行为放在一起；
- 构造函数：创建节点时建立订阅；
- 成员变量：保存订阅和消息计数；
- 成员函数：收到消息后执行；
- `const`：承诺只读取消息；
- `std::size_t`：安全表示数量；
- lambda：把“收到消息后的动作”交给 ROS 2。

这些不是单独的语法作业；它们正好构成一个 ROS 2 C++ 订阅节点。

## 遇到问题怎么办

### `Package 'predictive_nav_perception' not found`

说明当前终端没有加载刚构建的包。确认在构建成功后执行了：

```bash
source /home/aderfd/PredictiveNav2/install/setup.bash
```

### 节点只显示 `Waiting for ...`，没有 `scan #10`

确认 Gazebo 仍在另一个终端运行，并执行：

```bash
ros2 topic info /scan -v
```

若 `/scan` 存在但节点仍无输出，把 `ros2 topic info /scan -v` 的内容和节点终端的全部输出发给我。

### 构建报错

不要自己删除或改动 CMake。把构建输出从第一条 `error:` 到最后一行发给我；C++ 编译报错初看很长是正常的，我们会只找第一条真正原因。

## 这一小步的完成标准

- [ ] 构建成功，显示 `1 package finished`。
- [ ] `ros2 run predictive_nav_perception scan_info_node` 能启动。
- [ ] 持续出现 `scan #10`、`scan #20` 等日志。
- [ ] 日志中的 `frame` 是 `lidar_link`，且 `ranges` 数量大于 0。
- [ ] 我能用一句话解释：这个节点只订阅并观察 `/scan`，尚未识别障碍物。

当你看到第一行真实日志后，把它复制或截图给我。下一步会利用同一条消息中的 `ranges`，先学习怎样安全筛选无效距离。
