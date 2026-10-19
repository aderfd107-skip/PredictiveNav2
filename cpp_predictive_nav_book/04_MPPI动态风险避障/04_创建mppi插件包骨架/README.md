# 04：创建 MPPI 插件包骨架

> **本步目标：** 创建一个独立、可构建的 `predictive_nav_mppi` 插件包；此时还不订阅预测，也不计算风险。
>
> **AI 会做什么：** 生成目录、`package.xml`、`CMakeLists.txt` 与空类骨架，并在 diff 中说明每个依赖的用途。
>
> **你只需要做到：** 执行构建命令并理解“包能编译”与“插件能被 Nav2 发现”是两件不同的事。

新建 `predictive_nav_mppi` C++ 包，加入 Nav2 MPPI、pluginlib 和 ROS 2 的正确依赖，先完成可构建空插件。

**插件**是主程序按约定在运行时加载的扩展代码；独立包能避免修改 Nav2 源码。

完成标准：包可构建，且尚未改变任何导航行为。

## 本步 AI 生成范围与理解目标

AI 直接生成 `package.xml`、`CMakeLists.txt`、目录和空源文件。你不必手工建立文件，也不必背 CMake；但要核对包名、库目标名、依赖列表和文件组织为什么这样设计。

```text
src/predictive_nav_mppi/
├── CMakeLists.txt
├── package.xml
├── include/predictive_nav_mppi/
│   └── dynamic_risk_critic.hpp
├── src/
│   └── dynamic_risk_critic.cpp
└── predictive_nav_mppi_critics.xml
```

## 本步需要的依赖

先列出这些依赖并查明每个用途：

```text
ament_cmake                 构建 ROS 2 C++ 包
rclcpp                      logger、参数、subscription
nav2_mppi_controller        CriticFunction 和 CriticData
nav2_costmap_2d             Critic 基类传入的 costmap 环境
pluginlib                    运行时加载插件
predictive_nav_msgs         PredictedTrajectoryArray（第 07 步开始真正使用）
```

AI 生成的 CMake 会包含：`find_package(...)`、`add_library(...)`、`target_include_directories(...)`、`ament_target_dependencies(...)`、`install(...)`。第 06 步再补 pluginlib export；你只需逐项确认它们的作用。

## 本步验收

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_mppi
```

第一次只要求库能构建；此时不把它加进 Nav2 YAML，不创建控制 topic，也不改变机器人行为。

## 实施顺序：AI 将创建哪些空文件

AI 在 `src/` 下创建目录，绝不修改 `/opt/ros` 中的 Nav2 源码：

```bash
cd ~/PredictiveNav2
mkdir -p src/predictive_nav_mppi/include/predictive_nav_mppi
mkdir -p src/predictive_nav_mppi/src
touch src/predictive_nav_mppi/CMakeLists.txt
touch src/predictive_nav_mppi/package.xml
touch src/predictive_nav_mppi/include/predictive_nav_mppi/dynamic_risk_critic.hpp
touch src/predictive_nav_mppi/src/dynamic_risk_critic.cpp
touch src/predictive_nav_mppi/predictive_nav_mppi_critics.xml
```

此节只生成 `package.xml` 与 `CMakeLists.txt` 的最小构建信息。你要读懂 CMake 的目标关系：

```text
dynamic_risk_critic.cpp
       ↓ 编译成
libpredictive_nav_mppi.so
       ↓ 安装到
install/predictive_nav_mppi/lib/
```

先不写 plugin XML 内容，也不把库加入 YAML；那是第 06 节的工作。

### CMake 自检问题

你在写每一段 CMake 前先回答：

- `find_package(nav2_mppi_controller REQUIRED)` 是让编译器找到什么？
- `add_library(...)` 生成的是可执行文件还是 `.so` 动态库？
- 为什么 public include 路径包含 `include/`，但不把 `src/` 暴露出去？
- `install(TARGETS ...)` 缺失时，为什么工作空间 source 后 Nav2 仍找不到库？

### 本节交付物

停止在“库可编译”这里。保存目录树、`package.xml`、`CMakeLists.txt` 与第一次 build 输出；不要为了消除报错而提前让 AI 生成第 05、06 节的代码。
