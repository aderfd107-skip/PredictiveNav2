# 05：定义 DynamicRiskCritic 插件

创建继承 Nav2 MPPI critic 基类的 `DynamicRiskCritic`，只实现生命周期入口和空评分函数。

**继承**表示新类复用 Nav2 规定的接口；它不是复制粘贴控制器代码。空评分阶段先证明接口正确，再逐项增加逻辑。

完成标准：代码能说明初始化、评分和清理分别在哪发生，且空 critic 不改变结果。
