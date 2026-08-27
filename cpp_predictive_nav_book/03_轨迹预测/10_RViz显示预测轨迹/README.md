# 10：RViz 显示预测轨迹

把预测点、连线和不确定性范围作为独立调试 Marker 显示在 RViz。

**Marker** 只用于人看结果，不能反过来成为算法输入。它应与 LiDAR cluster、真实 Gazebo Marker 区分颜色和 topic。

完成标准：能在 RViz 对照当前目标、未来轨迹和不确定性，并截图保存证据。
