# 01：观察 cluster 消息

目的：确认第 01 模块输出的 `/dynamic_obstacles/clusters` 真的是第 02 模块唯一输入。

你将用 `ros2 interface show` 和 `ros2 topic echo --once` 看清：数组共同的 `header.stamp`、`header.frame_id=odom`，以及每个 cluster 的中心、尺寸和点数。

这一步还会确认传感器 QoS。跟踪节点必须使用兼容的 `SensorDataQoS`，否则代码正确也会收不到数据。

完成时你能解释：为什么 tracking 使用 `/dynamic_obstacles/clusters`，而不读取 RViz Marker 或 Gazebo 真值。
