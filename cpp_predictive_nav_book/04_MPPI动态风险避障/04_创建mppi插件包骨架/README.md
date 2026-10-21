# 04：创建 MPPI 插件包骨架

> 本步目标：创建一个可构建的 `predictive_nav_mppi` 共享库；暂时不定义 Critic 类、不注册 pluginlib、不改变导航。

## 为什么单独建包

不修改 `/opt/ros/jazzy` 中的 Nav2 源码，而是在工作空间内创建扩展包：

```text
src/predictive_nav_mppi/
├── CMakeLists.txt
├── package.xml
├── include/predictive_nav_mppi/dynamic_risk_critic.hpp
├── src/dynamic_risk_critic.cpp
└── predictive_nav_mppi_critics.xml
```

这样 Nav2 升级时不需要维护私有补丁，自己的 Critic 也能独立编译、安装和测试。

## `CMakeLists.txt` 做了什么

当前文件创建了一个共享库：

```cmake
add_library(${PROJECT_NAME} SHARED
  src/dynamic_risk_critic.cpp
)
```

它会生成：

```text
libpredictive_nav_mppi.so
```

`find_package(...)` 与 `ament_target_dependencies(...)` 声明这个库将使用 ROS 2、Nav2 MPPI、
costmap、pluginlib 和本项目消息包。虽然第 04 步还没实际订阅 prediction，但提前声明
`predictive_nav_msgs` 可让后续步骤不用改变包的身份。

`install(TARGETS ...)` 会把 `.so` 放到 `install/predictive_nav_mppi/lib/`；没有它，工作空间
source 后其他 ROS 2 包无法找到该库。

## `package.xml` 做了什么

`package.xml` 是 ROS 2 包的说明书。它告诉 colcon：包叫 `predictive_nav_mppi`，构建工具是
ament_cmake，运行/构建依赖包含 `nav2_mppi_controller` 等包。

它不是 C++ include 列表；真正把依赖传给编译器的是 CMake。

## 如何构建

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_mppi --symlink-install
source install/setup.bash
ros2 pkg prefix predictive_nav_mppi
```

最后一条应输出工作空间下的 `install/predictive_nav_mppi`。还可以检查：

```bash
test -e install/predictive_nav_mppi/lib/libpredictive_nav_mppi.so && echo OK
```

## 这一步还没有做什么

- 没有 `DynamicRiskCritic` 类；
- XML 尚未写入 `<library>` 和 `<class>`；
- 没有 `PLUGINLIB_EXPORT_CLASS`；
- 没有 YAML 将它加入 MPPI。

所以“库可编译”不等于“Nav2 已发现插件”。第 06 步才会建立运行时加载链路。

## 本步完成标准

- [ ] `colcon build --packages-select predictive_nav_mppi` 成功；
- [ ] `libpredictive_nav_mppi.so` 位于 install 目录；
- [ ] 当前导航行为完全未变化；
- [ ] 知道 `.so` 是动态库，不是可以 `ros2 run` 的节点。

## 下一步

[05_定义DynamicRiskCritic插件](../05_定义DynamicRiskCritic插件/README.md)：在这个库中加入
真正继承 Nav2 Critic 接口的空类。
