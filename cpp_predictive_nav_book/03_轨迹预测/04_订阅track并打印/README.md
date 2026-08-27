# 04：订阅 track 并打印

prediction 节点订阅 `/dynamic_obstacles/tracks`，先只节流打印 ID、位置、速度、时间戳和协方差摘要。

这一步验证真正的输入已经连通；没有输入就不要提前写矩阵公式。

完成标准：动态场景中，终端持续收到 `odom` 下的 track，并能指出速度字段的单位是 `m/s`。
