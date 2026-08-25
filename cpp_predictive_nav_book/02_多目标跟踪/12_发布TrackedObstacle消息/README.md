# 12：发布 TrackedObstacle 消息

目的：扩展 `predictive_nav_msgs`，定义 `TrackedObstacle` 与 `TrackedObstacleArray`，并发布：

```text
/dynamic_obstacles/tracks
```

每条正式输出会带稳定 ID、`PoseWithCovariance`、`TwistWithCovariance`、尺寸、轨迹年龄、丢失次数和置信度。第 03 模块只订阅此 topic，不重新猜测 ID 或速度。
