# 09：发布预测结果

将每帧所有有效预测发布到 `/dynamic_obstacles/predictions`，使用与输入一致的时间和 `odom` 语义。

发布前要验证数组不含 NaN/Inf，并为“本帧没有有效预测”保留清晰行为。

完成标准：`ros2 topic echo --once /dynamic_obstacles/predictions` 能看到 ID 和多时刻未来点。
