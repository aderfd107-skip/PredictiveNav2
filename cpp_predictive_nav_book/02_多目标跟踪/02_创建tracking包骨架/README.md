# 02：创建 tracking 包骨架

## 这一步完成了什么

我们创建一个新的真实 ROS 2 C++ 包：

```text
src/predictive_nav_tracking/
├── CMakeLists.txt
├── package.xml
└── src/
    └── tracking_node.cpp
```

它将来专门负责多目标跟踪。现在它只会启动并打印一行日志；**不会**订阅 cluster、创建 ID、计算速度或运行卡尔曼滤波。

这看似很小，但它先把工程边界建立正确了：

```text
predictive_nav_perception   从 /scan 得到“这一帧看到了什么”
predictive_nav_tracking     从连续 cluster 得到“哪个一直是同一个物体”
```

两个包通过 `/dynamic_obstacles/clusters` 消息连接，而不是让跟踪代码直接读取感知包内部的 C++ 变量。

## 为什么不继续修改 `scan_info_node`

如果把聚类、跟踪、预测、Nav2 风险评分全塞进同一个节点，后面会出现三个问题：

1. 一个错误很难定位：不知道是激光聚类错了，还是 ID 关联错了；
2. 无法单独复用或回放输入：跟踪器不能只拿 cluster bag 调试；
3. 代码会越来越长，初学时更难读。

因此本项目按职责拆开：感知输出单帧观测，跟踪维护跨帧状态，预测使用 track 的速度与协方差，MPPI 最后才消费预测结果。

## 新增的三个文件分别做什么

### `package.xml`：声明“我是谁、我依赖谁”

它写了：

```xml
<name>predictive_nav_tracking</name>
<depend>rclcpp</depend>
<depend>predictive_nav_msgs</depend>
<depend>eigen</depend>
```

- `rclcpp`：让这个包能写 ROS 2 C++ node；
- `predictive_nav_msgs`：第 03 步要使用 `ObstacleClusterArray`；
- `eigen`：第 07 步开始需要矩阵处理卡尔曼状态和协方差。

这里的 Eigen 是 Linux 系统中的 C++ 数学库，不是一个要用 `ros2 run` 启动的节点。现在它还没被 C++ 代码使用，只是提前让构建系统知道后面会依赖它。

### `CMakeLists.txt`：声明“怎样构建成可运行程序”

最需要认识的几行是：

```cmake
add_executable(tracking_node src/tracking_node.cpp)
```

意思是：把 `src/tracking_node.cpp` 编译成一个名叫 `tracking_node` 的程序。

```cmake
ament_target_dependencies(tracking_node
  predictive_nav_msgs
  rclcpp
)
```

意思是：这个程序需要 ROS 2 C++ 和项目自定义消息包。

```cmake
install(TARGETS tracking_node DESTINATION lib/${PROJECT_NAME})
```

意思是：构建完成后，把程序放到 ROS 2 能查找的位置。正因为有这一段，你才能用：

```bash
ros2 run predictive_nav_tracking tracking_node
```

而不是手动去 `build/` 目录寻找可执行文件。

### `tracking_node.cpp`：最小、可运行的 C++ 节点

当前代码的核心是：

```cpp
class TrackingNode : public rclcpp::Node
```

读作：定义一个名为 `TrackingNode` 的类，它是一种 ROS 2 节点。

构造函数：

```cpp
TrackingNode()
: Node("tracking_node")
```

读作：创建节点时，在 ROS 2 图中把它命名为 `tracking_node`。

当前唯一动作是：

```cpp
RCLCPP_INFO(get_logger(), "...");
```

它只是向终端输出一条提示，证明程序的构造函数确实执行了。

最后的 `main()` 是每个 C++ 程序的入口：

```text
rclcpp::init()  → 启动 ROS 2
rclcpp::spin()  → 持续让节点接收消息（现在还没有订阅）
rclcpp::shutdown() → 正常退出 ROS 2
```

你现在不需要从空白写出这些代码。下一步会在这个已经能运行的节点中只增加“订阅 cluster”这一件事。

## 构建

打开一个新终端，逐行运行：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_tracking
source install/setup.bash
```

预期末尾包含：

```text
Finished <<< predictive_nav_tracking
Summary: 1 package finished
```

如果构建失败，不要自行删依赖或改 CMake；把从第一个 `Error` 开始的终端内容发给我。

## 运行最小节点

构建成功后，在同一终端运行：

```bash
ros2 run predictive_nav_tracking tracking_node
```

预期只出现一行：

```text
[INFO] [tracking_node]: Tracking package is running. Step 03 will subscribe to /dynamic_obstacles/clusters.
```

程序不会自动退出，因为 `rclcpp::spin()` 正在让节点保持运行。此时按 `Ctrl + C` 正常停止即可。

即使 Gazebo、RViz 和感知节点都没有运行，这一步的最小节点也能启动；因为它还没有订阅任何外部 topic。

## 本步完成标准

- [ ] `src/predictive_nav_tracking/` 中有 `package.xml`、`CMakeLists.txt`、`src/tracking_node.cpp`。
- [ ] `colcon build --packages-select predictive_nav_tracking` 成功。
- [ ] `ros2 run predictive_nav_tracking tracking_node` 输出启动提示。
- [ ] 我能说出：感知包负责单帧 cluster，跟踪包负责跨帧 Track。

## 本章关联的 ROS 2 / C++ 知识

**本章必须懂**：一个 ROS 2 C++ 包至少有 `package.xml`（依赖与身份）、`CMakeLists.txt`（构建与安装规则）和源码。`rclcpp::spin()` 让节点持续运行，等待未来的订阅回调。

**可选扩展**：现在不要求掌握 CMake 全部语法、继承的完整理论，或 Eigen 矩阵操作。你只需要知道它们分别在“构建”“ROS 2 节点”“后续卡尔曼滤波”中起什么作用。

下一步会在不改变这个包结构的前提下，为 `tracking_node` 添加 `/dynamic_obstacles/clusters` 订阅者。
