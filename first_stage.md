第一部分：场景和功能边界
第二部分：机器人模型
第三部分：地图、Gazebo、RViz
第四部分：单机器人和双机器人 Nav2 基础
第五部分：任务系统和任务分配
第六部分：无冲突路径规划
第七部分：执行反馈、评测和文档

1. 确定具体场景
2. 定义第一版功能边界
3. 设计任务系统
4. 设计机器人模型
5. 建 Gazebo world 和已知地图
6. 配 RViz 可视化
7. 单机器人 Nav2 跑通
8. 双机器人 Nav2 namespace 跑通
9. 离线实现 A* 和任务分配
10. 离线实现 reservation table / prioritized planning
11. 接入 ROS 2：task manager / allocator / planner / executor
12. 多机器人执行服务任务
13. 记录评测数据
14. 总结第一阶段问题，决定是否进入 SLAM 探索扩展



地图
办公楼一层
├── 前台 / 服务台
├── 会议室 A
├── 会议室 B
├── 办公区
├── 设备间
├── 打印区
├── 小仓储间
├── 主走廊
├── 窄通道
├── T 字路口
└── 充电区


机器人建模

第一阶段采用两台同底盘、异外观的通用室内移动服务机器人。

总体原则：

- 两台机器人外观不同，便于展示和区分。
- 两台机器人第一阶段任务能力相同，都能执行点检任务和配送任务。
- 第一阶段不做真实抓取、开柜、视觉识别和异构能力约束。
- 后续阶段再扩展为真正的异构机器人调度。

Robot 1：Delivery Service Robot

定位：

- 偏配送外观的楼宇服务机器人。

模型特征：

- 矩形差速底盘
- 蓝色主色
- 上方半封闭货箱 / 托盘
- 前方 2D LiDAR
- IMU
- 顶部摄像头外观
- 编号 robot_1

第一阶段任务能力：

- inspect_point
- deliver_item

Robot 2：Inspection Service Robot

定位：

- 偏点检 / 响应外观的楼宇服务机器人。

模型特征：

- 矩形差速底盘
- 橙色主色
- 顶部传感器杆
- 摄像头外观
- 前方 2D LiDAR
- IMU
- 编号 robot_2

第一阶段任务能力：

- inspect_point
- deliver_item

说明：

- 第一阶段 robot_2 虽然外观偏点检，但仍允许执行配送任务。
- 这样可以先把多机器人任务调度、无冲突路径规划和 Nav2 执行跑通。
- 第二阶段再引入能力约束，例如 delivery_robot 才能执行 deliver_item，inspection_robot 才能执行 inspect_point / respond_event。

当前模型包：

```text
src/mrt_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── display.launch.py
│   ├── display_delivery.launch.py
│   └── display_inspection.launch.py
├── rviz/
│   └── display.rviz
└── urdf/
    ├── base.xacro
    ├── gazebo_plugins.xacro
    ├── inertials.xacro
    ├── materials.xacro
    ├── payload.xacro
    ├── robot.urdf.xacro
    ├── sensors.xacro
    └── wheels.xacro
```

建模边界：

必须支持：

- 差速移动底盘
- 2D LiDAR
- IMU
- odom / cmd_vel 接口预期
- Nav2 需要的 TF 结构
- 货箱模块
- 传感器杆模块
- 颜色区分
- 多机器人 namespace 扩展

暂时不做：

- 机械臂
- 真实抓取
- 真实货物装卸
- 柜门动画
- 屏幕 UI
- 真实摄像头识别
- 不同底盘类型
- 真正异构任务能力约束
