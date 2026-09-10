# 06：导出并加载插件

编写 plugin XML 和 CMake 导出规则，再在 MPPI 参数中声明该 critic，让 Nav2 真正加载它。

**导出**是把 C++ 类名告诉 pluginlib；代码编译成功不等于 Nav2 一定找得到它。

完成标准：启动日志明确显示 DynamicRiskCritic 已加载，且加载失败时能定位 XML、库名或参数名。

## 本步手写范围

**必须亲手写一遍：** plugin XML 的 class 条目、导出宏、MPPI YAML 中的 critic 名称。它们是插件能否被发现的三段链路。

**需要理解，可参考：** CMake 的 `pluginlib_export_plugin_description_file(...)` 具体语法。不要猜库名，后面通过构建和日志验证。

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

你先自己补全下面的 `???`：

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
