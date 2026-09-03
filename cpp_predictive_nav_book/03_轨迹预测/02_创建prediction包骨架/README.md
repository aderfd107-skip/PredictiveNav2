# 02：创建 prediction 包骨架

> 本步目标：新建一个能被 ROS 2 构建和运行的 C++ 包；**暂时不订阅、不发布、更不做任何预测计算**。

## 1. 为什么还要新建一个包？

第二模块的 `predictive_nav_tracking` 已经负责一件很明确的事：从每帧 cluster 中维护目标的当前位置和速度，输出 `/dynamic_obstacles/tracks`。

第三模块要做的是另一件事：读取 track，把“现在的位置和速度”推到未来多个时刻，最终输出 `/dynamic_obstacles/predictions`。它不应该塞进 `tracking_node`，原因是：

- **职责清楚**：跟踪出错时查 tracking；预测效果差时查 prediction。
- **能独立回放**：以后可以只回放 `/dynamic_obstacles/tracks`，专门调试预测。
- **能替换算法**：以后将 CV 模型换成更复杂的模型时，不必改动跟踪模块。

这里的“包（package）”可以先理解为 ROS 2 中一块可以独立构建、安装、运行的代码单元。一个包通常至少要有：

```text
src/predictive_nav_prediction/
├── package.xml                 # 包的身份证：名字、依赖、许可证等
├── CMakeLists.txt              # 怎样把 C++ 源码编译成可执行程序
└── src/
    └── prediction_node.cpp     # 未来预测节点的主程序
```

## 2. 本步新增了什么？

已经创建：

- 包名：`predictive_nav_prediction`
- 可执行程序名：`prediction_node`
- 节点名：`prediction_node`
- 当前唯一依赖：`rclcpp`

注意三个名字现在恰好一样，但它们属于不同层级：

| 名称 | 是什么 | 本项目中的值 |
| --- | --- | --- |
| 包名 | `colcon build`、`ros2 run` 用来定位代码包 | `predictive_nav_prediction` |
| 可执行程序名 | 被编译出来的 Linux 程序 | `prediction_node` |
| 节点名 | 程序运行后出现在 ROS 图里的名字 | `prediction_node` |

以后运行的完整格式就是：

```bash
ros2 run <包名> <可执行程序名>
```

所以本项目是：

```bash
ros2 run predictive_nav_prediction prediction_node
```

## 3. 先读懂 `prediction_node.cpp`

现在的代码故意很短：

```cpp
class PredictionNode : public rclcpp::Node
```

这表示“定义一个叫 `PredictionNode` 的 C++ 类，它是 ROS 2 节点 `rclcpp::Node` 的一种”。你可以把 `class` 暂时看作 C 语言中 `struct` 的升级版：它不仅可以保存数据，还可以把相关功能放在一起。

构造函数中的：

```cpp
: Node("prediction_node")
```

是在创建这个节点时，把 ROS 图中的节点名设为 `prediction_node`。而：

```cpp
RCLCPP_INFO(get_logger(), "...");
```

只是向终端打印一条 ROS 2 日志，用来证明程序确实被启动了。

`main()` 中三句最重要的话是：

```cpp
rclcpp::init(argc, argv);                    // 初始化 ROS 2
rclcpp::spin(std::make_shared<PredictionNode>());  // 创建节点并持续运行
rclcpp::shutdown();                          // Ctrl+C 后清理 ROS 2 资源
```

其中 `spin()` 会一直等待 ROS 2 的事件，所以运行节点后终端不会自动返回；这是正常现象。按 `Ctrl+C` 才会结束它。

## 4. `CMakeLists.txt` 和 `package.xml` 分别做什么？

### `package.xml`：告诉 ROS 2 “我需要谁”

本步只有：

```xml
<depend>rclcpp</depend>
```

因为目前代码只用了 ROS 2 C++ 客户端库 `rclcpp`。还没有使用 `TrackedObstacleArray`，因此暂时**不写** `predictive_nav_msgs` 依赖。第 04 步真正订阅 `/dynamic_obstacles/tracks` 时再添加，避免“代码没用到却先堆依赖”。

### `CMakeLists.txt`：告诉编译器“怎样编译”

核心三段是：

```cmake
find_package(rclcpp REQUIRED)
add_executable(prediction_node src/prediction_node.cpp)
ament_target_dependencies(prediction_node rclcpp)
```

它们依次表示：找到 `rclcpp`、把 `.cpp` 编译成 `prediction_node`、让这个程序在编译和运行时正确使用 `rclcpp`。

最后 `install(...)` 很重要：`colcon build` 不只是把程序放在 `build/`，还要安装到 `install/`。`ros2 run` 找的是安装后的程序；少了这段，编译也许成功，但 ROS 2 找不到可执行程序。

## 5. 现在请你自己构建并运行

在一个终端执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node
```

预期看到：

```text
[INFO] [...] [prediction_node]: prediction_node started. Step 02 only creates the package skeleton.
```

此时它没有输入话题、没有输出话题，也不会让小车移动；这正是本步应有的状态。确认日志后按 `Ctrl+C` 退出。

若提示 `Package 'predictive_nav_prediction' not found`，通常是忘记了本终端中的：

```bash
source install/setup.bash
```

若提示找不到 `prediction_node`，先重新执行构建命令，再 source 一次 `install/setup.bash`。

## 6. 完成标准

- [ ] `colcon build --packages-select predictive_nav_prediction` 成功。
- [ ] `ros2 run predictive_nav_prediction prediction_node` 能输出启动日志。
- [ ] 能说出：这个节点目前只是“预测模块的空房子”，还没有读取 track，也没有预测任何目标。

## 本章必须懂的 ROS 2 / C++ 知识

- ROS 2 包、可执行程序、节点是三个不同概念。
- `package.xml` 管依赖声明，`CMakeLists.txt` 管 C++ 的编译和安装。
- `rclcpp::spin()` 会让节点持续运行，等待后续的订阅回调、定时器等事件。
- 一个新包应先能独立构建和启动，再逐步添加消息、订阅与算法；出错时定位范围会小得多。

## 下一步

[03_定义预测消息](../03_定义预测消息/README.md)：目前 tracker 只能告诉我们目标的“现在”。下一步先把“一个目标在多个未来时刻的位置与不确定性”定义成正式 ROS 2 消息。
