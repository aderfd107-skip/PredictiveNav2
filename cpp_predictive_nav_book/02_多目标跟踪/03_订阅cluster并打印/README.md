# 03：订阅 cluster 并打印

目的：让 `tracking_node` 使用 `SensorDataQoS` 订阅 `/dynamic_obstacles/clusters`，只打印一帧的时间、frame、cluster 数量和第一个中心点。

此时节点不分配 ID、不估计速度。它只证明第 01 与第 02 模块的正式接口已真正接通。

完成标准是：感知节点运行时，跟踪节点持续收到 `frame=odom` 的 cluster 数组。
