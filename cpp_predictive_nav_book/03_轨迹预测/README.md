# 第三模块：从轨迹状态到未来预测

## 这块要解决什么

第二模块的 `track` 说的是“障碍物现在在哪里、速度大约是多少”。本模块要回答“如果它暂时保持这种运动，未来 0.2、0.4、0.6 秒可能在哪里，以及这个判断有多不确定”。

这里的**轨迹预测**不是控制小车，也不是保证看穿未来；它只是把 track 的状态按短时间模型向前推算，并发布给下一模块的风险评估使用。

## 完成后的数据流

```text
/dynamic_obstacles/tracks
        ↓
predictive_nav_prediction
        ↓
/dynamic_obstacles/predictions
        ↓
第 04 模块的 DynamicRiskCritic
```

## 步骤路线

1. [00_这一模块要做什么](00_这一模块要做什么/README.md)
2. [01_观察tracks输入](01_观察tracks输入/README.md)
3. [02_创建prediction包骨架](02_创建prediction包骨架/README.md)
4. [03_定义预测消息](03_定义预测消息/README.md)
5. [04_订阅track并打印](04_订阅track并打印/README.md)
6. [05_确定预测时间轴](05_确定预测时间轴/README.md)
7. [06_CV传播未来位置](06_CV传播未来位置/README.md)
8. [07_传播位置不确定性](07_传播位置不确定性/README.md)
9. [08_处理过期与异常预测](08_处理过期与异常预测/README.md)
10. [09_发布预测结果](09_发布预测结果/README.md)
11. [10_RViz显示预测轨迹](10_RViz显示预测轨迹/README.md)
12. [11_rosbag回放与排错](11_rosbag回放与排错/README.md)

## 先记住的边界

- 预测只使用第 02 模块发布的 track，不读取 Gazebo 真值。
- CV 是短时间近似；方块转向、被遮挡时，预测自然会变差。
- 本模块只提供“风险信息”，不直接发布 `/cmd_vel`。
