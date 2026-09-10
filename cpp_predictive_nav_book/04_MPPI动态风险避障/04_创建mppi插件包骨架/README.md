# 04：创建 MPPI 插件包骨架

新建 `predictive_nav_mppi` C++ 包，加入 Nav2 MPPI、pluginlib 和 ROS 2 的正确依赖，先完成可构建空插件。

**插件**是主程序按约定在运行时加载的扩展代码；独立包能避免修改 Nav2 源码。

完成标准：包可构建，且尚未改变任何导航行为。

## 本步手写范围

**需要理解，可参考：** `package.xml`、`CMakeLists.txt` 的固定格式。你不必背下来，但必须亲手建立并通过第一次构建。

**必须手写：** 包名、库目标名、依赖列表和文件组织。建议结构：

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

你需要在 CMake 中自己完成的关键词是：`find_package(...)`、`add_library(...)`、`target_include_directories(...)`、`ament_target_dependencies(...)`、`install(...)`。第 06 步再补 pluginlib export。

## 本步验收

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_mppi
```

第一次只要求库能构建；此时不把它加进 Nav2 YAML，不创建控制 topic，也不改变机器人行为。
