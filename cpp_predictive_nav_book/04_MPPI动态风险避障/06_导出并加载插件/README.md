# 06：导出并加载插件

> 本步目标：让 controller_server 在运行时找到并实例化空的 DynamicRiskCritic；仍不加入任何动态风险公式。

## “编译成功”为什么还不够

C++ 已能生成 `libpredictive_nav_mppi.so`，但 Nav2 仍不知道其中哪个类可作为 Critic。
pluginlib 需要三段信息：

```text
DynamicRiskCritic C++ 类
        ↓ EXPORT_CLASS
库中导出的具体类型
        ↓ XML
该类型属于哪个动态库、继承哪个基类
        ↓ CMake / package.xml 安装与注册
ament index 能找到 XML
        ↓ MPPI YAML
controller_server 请求加载该 Critic
```

任何一段缺失，都可能出现“编译成功但运行时找不到类”。

## XML 应表达什么

`predictive_nav_mppi_critics.xml` 现在包含一个库和一个类。逻辑上等价于：

```xml
<library path="predictive_nav_mppi">
  <class
    name="mppi::critics::DynamicRiskCritic"
    type="mppi::critics::DynamicRiskCritic"
    base_class_type="mppi::critics::CriticFunction" />
</library>
```

`type` 必须与 C++ namespace 和类名完全一致；`base_class_type` 必须是 Nav2 要加载的基类。
Jazzy 的 `CriticManager` 会把 YAML 中的 `DynamicRiskCritic` 自动扩展成
`mppi::critics::DynamicRiskCritic`，因此这个插件的**C++ namespace 必须使用
`mppi::critics`**。这不是包名：ROS 包、动态库和源码目录仍然是
`predictive_nav_mppi`。

## YAML 怎样加入空 Critic

不要修改 baseline。已创建 `nav2_mppi_dynamic_risk.yaml`，在 `FollowPath.critics` 最后加入：

```yaml
critics: [..., "DynamicRiskCritic"]

DynamicRiskCritic:
  enabled: true
```

空 Critic 加载成功后，导航行为应和 baseline 基本一致；它只有日志，不增加 cost。

## 如何验证

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select predictive_nav_mppi predictive_nav_bringup --symlink-install

ros2 launch predictive_nav_bringup predictive_dynamic_lab_mppi.launch.py \
  params_file:=$(ros2 pkg prefix predictive_nav_bringup)/share/predictive_nav_bringup/config/nav2_mppi_dynamic_risk.yaml
```

启动动态风险配置后，在 controller_server 日志中查找：

```text
Initialized DynamicRiskCritic as an empty MPPI critic
```

若没有该日志，不能直接进入 prediction 步骤。先检查 XML、导出宏、安装规则、YAML 中的
Critic 名称，以及 launch 是否真的加载了动态版本 YAML。

## 常见问题

### `Class ... does not exist`

通常是 XML 的 `type`、`PLUGINLIB_EXPORT_CLASS` 或 namespace 不一致。

### 没有报错，但没有初始化日志

通常是该 Critic 没有写入 `FollowPath.critics`，或 launch 仍使用 baseline YAML。

## 本步完成标准

- [x] XML、导出宏、CMake、package.xml 和 YAML 名称一致；
- [x] controller_server 日志确认空 Critic 已加载；
- [ ] 加载空 Critic 前后导航行为相同；
- [ ] baseline YAML 仍可以独立启动。

## 下一步

[07_缓存预测障碍物](../07_缓存预测障碍物/README.md)：插件已经能运行，下一步才订阅
`/dynamic_obstacles/predictions`。
