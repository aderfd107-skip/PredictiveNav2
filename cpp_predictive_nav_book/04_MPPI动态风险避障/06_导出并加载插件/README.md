# 06：导出并加载插件

> **本步目标：** 让 Nav2 能在运行时发现并加载空的 `DynamicRiskCritic`；此时仍不计算动态风险。
>
> **AI 会做什么：** 生成 plugin XML、导出宏、CMake 安装规则和 YAML 的最小改动，并解释类名、库名、XML 条目之间的对应关系。
>
> **你只需要做到：** 用构建与启动日志区分“代码编译通过”“动态库被安装”“运行时插件被加载”。

编写 plugin XML 和 CMake 导出规则，再在 MPPI 参数中声明该 critic，让 Nav2 真正加载它。

**导出**是把 C++ 类名告诉 pluginlib；代码编译成功不等于 Nav2 一定找得到它。

完成标准：启动日志明确显示 DynamicRiskCritic 已加载，且加载失败时能定位 XML、库名或参数名。

## 本步 AI 生成与审读

AI 生成 plugin XML、导出宏和 MPPI YAML 差分。你要检查具体类名、库名、XML 条目和 YAML critic 名称完全一致；它们是插件能否被发现的三段链路。CMake 的 `pluginlib_export_plugin_description_file(...)` 可由 AI 填写，但必须通过构建和日志验证。

## 需要的新增内容

### 1. cpp 文件末尾的导出宏

```cpp
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  predictive_nav_mppi::DynamicRiskCritic,
  mppi::critics::CriticFunction)
```

这里第一个名字是你的具体类，第二个名字是 Nav2 通过 pluginlib 查找的基类；两者任何一个 namespace 写错，库即使编译成功也无法加载。

### 2. `predictive_nav_mppi_critics.xml`

AI 会根据实际 CMake 库目标补全下面的 `path`；你要核对它不是源码文件名：

```xml
<class_libraries>
  <library path="???">
    <class
      type="predictive_nav_mppi::DynamicRiskCritic"
      base_class_type="mppi::critics::CriticFunction">
      <description>Scores time-aligned dynamic obstacle risk.</description>
    </class>
  </library>
</class_libraries>
```

`path` 对应 CMake 生成并安装的库名，不是源码文件名，也通常不写 `lib` 前缀或 `.so` 后缀。

### 3. MPPI YAML

在专用 MPPI 配置中把插件别名加入 critic 列表，并创建同名参数空间：

```yaml
FollowPath:
  critics: ["...", "DynamicRiskCritic"]
  DynamicRiskCritic:
    enabled: true
```

## 排错顺序

```text
库未生成       → CMake target / 依赖问题
XML 未安装      → install() / export 宏问题
找不到类        → XML type、namespace、导出宏问题
类能加载但无日志 → YAML critic 名称或 namespace 问题
```

本步暂时不写风险公式。你要保留一次插件成功加载的完整 controller_server 日志，这是后续面试中很好的工程证据。

## 生成与验证顺序：三段链必须逐段通

插件发现不是一个动作，而是三段都正确：

```text
C++ 类 + EXPORT_CLASS
        ↓
XML：类名属于哪个动态库
        ↓
package.xml / CMake：把 XML 安装并注册到 ament index
        ↓
MPPI YAML：要求加载这个 Critic
```

AI 会补齐这两处构建声明；你需要逐字核对名称是否完全一致：

```cmake
# CMakeLists.txt，路径是 XML 文件相对于 package 根目录
pluginlib_export_plugin_description_file(
  nav2_mppi_controller predictive_nav_mppi_critics.xml)
```

```xml
<!-- package.xml 的 <export> 内，由 AI 生成后审读 -->
<nav2_mppi_controller plugin="${prefix}/predictive_nav_mppi_critics.xml" />
```

然后执行：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_mppi --symlink-install
source install/setup.bash
```

### 如何确认不是“编译成功但没安装”

```bash
find install/predictive_nav_mppi -name '*dynamic_risk*' -o -name '*critics.xml'
```

应至少看到动态库和 XML。之后才允许把 `DynamicRiskCritic` 加入 MPPI YAML。第一次加载时它是空评分：若添加后机器人明显改变行为，先停下并检查配置，而不是继续第 07 节。

## 让 AI 审查时提供的最小集合

`CMakeLists.txt` 的库与 install 段、`package.xml` 的 export 段、XML、`PLUGINLIB_EXPORT_CLASS` 所在的几行，以及 controller_server 第一次加载日志。
