# 06：导出并加载插件

编写 plugin XML 和 CMake 导出规则，再在 MPPI 参数中声明该 critic，让 Nav2 真正加载它。

**导出**是把 C++ 类名告诉 pluginlib；代码编译成功不等于 Nav2 一定找得到它。

完成标准：启动日志明确显示 DynamicRiskCritic 已加载，且加载失败时能定位 XML、库名或参数名。
