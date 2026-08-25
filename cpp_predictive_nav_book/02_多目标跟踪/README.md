# 第二模块：从障碍物候选观测到多目标轨迹

## 这一模块接在第 01 模块哪里

第 01 模块已经持续发布：

```text
/dynamic_obstacles/clusters
  header.stamp / header.frame_id=odom
  clusters[]：centroid、size_x_m、size_y_m、point_count
```

但每一帧的 cluster 都没有名字。例如，上一帧第 2 个 cluster 与下一帧第 2 个 cluster，不一定是同一个物体；数组下标会随视野、遮挡和聚类结果改变。

本模块要新增 `predictive_nav_tracking` C++ 包，最终形成：

```text
/dynamic_obstacles/clusters
  ↓
predictive_nav_tracking
  ↓
/dynamic_obstacles/tracks
  每条轨迹：稳定 ID、位置、速度、协方差、尺寸、命中次数、丢失次数
```

## 最终要解决的真实问题

连续两帧中，机器人看到三个 cluster：

```text
时刻 t：      A(1.0, 2.0)    B(3.0, 1.0)    C(5.0, 4.0)
时刻 t + dt： A(1.1, 2.0)    B(3.0, 1.0)    C(4.9, 4.0)
```

跟踪器要做的不是把数组第 0、1、2 项机械地对应起来，而是回答：

```text
哪个新观测最可能属于已有 track #17？
哪个观测是新出现的物体？
哪个旧轨迹这次暂时没看到？
```

每条轨迹使用恒定速度（CV，Constant Velocity）状态：

```text
x = [位置 x, 位置 y, 速度 vx, 速度 vy]
```

卡尔曼滤波会将上一帧状态的预测与当前激光观测结合起来，避免直接用两帧坐标相减而产生非常抖动的速度。

## 小步骤顺序

1. [00_这一模块要做什么](00_这一模块要做什么/README.md)：先分清 cluster、track、prediction 的职责。
2. [01_观察cluster消息](01_观察cluster消息/README.md)：不写代码，确认第 01 模块的正式输入接口。
3. [02_创建tracking包骨架](02_创建tracking包骨架/README.md)：建立独立的 C++ 跟踪包。
4. [03_订阅cluster并打印](03_订阅cluster并打印/README.md)：确认跟踪节点收到的是 `odom` 中、带时间戳的观测。
5. [04_定义Track轨迹数据](04_定义Track轨迹数据/README.md)：明确一条轨迹需要保存哪些长期状态。
6. [05_处理时间戳与dt](05_处理时间戳与dt/README.md)：不假定固定 10 Hz，使用真实测量时间计算 `dt`。
7. [06_单目标连续观测与朴素速度](06_单目标连续观测与朴素速度/README.md)：先直观看懂“速度为何会抖”，为卡尔曼滤波做准备。
8. [07_引入CV卡尔曼状态](07_引入CV卡尔曼状态/README.md)：使用 Eigen 表示位置、速度和协方差。
9. [08_预测已有轨迹](08_预测已有轨迹/README.md)：让每条轨迹按真实 `dt` 走到当前时刻之前的预计位置。
10. [09_最近邻数据关联](09_最近邻数据关联/README.md)：用距离门限把 cluster 与预测后的轨迹匹配。
11. [10_卡尔曼更新与速度估计](10_卡尔曼更新与速度估计/README.md)：用匹配到的观测修正位置、速度与不确定性。
12. [11_新生轨迹与丢失管理](11_新生轨迹与丢失管理/README.md)：处理新物体、短暂遮挡和应删除的旧轨迹。
13. [12_发布TrackedObstacle消息](12_发布TrackedObstacle消息/README.md)：定义并发布正式 `/dynamic_obstacles/tracks` 接口。
14. [13_RViz验证与动态方块观察](13_RViz验证与动态方块观察/README.md)：在 RViz 显示 ID、速度箭头和轨迹状态。
15. [14_rosbag回放与排错](14_rosbag回放与排错/README.md)：录制、回放固定输入，复现 ID 切换和参数问题。

## 本模块刻意不做什么

- 不直接用 Gazebo actor 的位置、速度或橙色真值 Marker；
- 不把“cluster 数组下标相同”误当作同一个物体；
- 不一开始实现匈牙利算法、JPDA、深度学习跟踪或复杂 DBSCAN；
- 不在本模块预测未来很久，也不修改 Nav2 控制器。

第一版采用可解释、可调试的**预测后最近邻贪心匹配 + CV 卡尔曼滤波**。当它稳定运行后，才值得用 benchmark 比较匈牙利算法等替代方案。

## 本模块完成时，你应该能说清

1. cluster 是单帧几何观测，track 是跨帧维护的状态；
2. 为什么必须使用消息时间戳计算 `dt`，而不是写死 `0.1 s`；
3. 为什么先预测，再关联，再更新；
4. 为什么一个暂时没被观测到的 track 不能立刻删除；
5. track 的速度与协方差如何成为第 03 模块轨迹预测的输入。

现在先从第 00 步开始。每一步的代码都会在实际开始该步时生成；你不需要提前手写任何卡尔曼滤波或 C++。
