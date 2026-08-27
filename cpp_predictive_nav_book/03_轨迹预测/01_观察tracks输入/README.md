# 01：观察 tracks 输入

先不写预测代码。确认第 02 模块真正发布的 `/dynamic_obstacles/tracks` 有稳定 ID、`odom` 坐标系、时间戳、位置、速度、协方差和尺寸。

**输入接口**就是两个模块约定的数据格式；先检查它，能避免预测模块建立在猜测字段上。

完成标准：能用 `ros2 topic echo --once` 和 `ros2 interface show` 看懂一条 track 的字段。
