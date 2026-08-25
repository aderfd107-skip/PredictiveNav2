# 02：创建 tracking 包骨架

目的：新建独立 ROS 2 C++ 包 `predictive_nav_tracking`，而不是把跟踪逻辑继续塞进感知节点。

感知包负责从 `/scan` 形成单帧观测；跟踪包负责跨帧状态。两个包通过消息接口连接，彼此可以独立调试和替换。

本步只创建 `package.xml`、`CMakeLists.txt` 和最小节点入口，并声明对 `rclcpp`、`predictive_nav_msgs`、Eigen 的依赖。不会实现跟踪算法。
