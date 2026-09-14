# 真实 2WD 机器人几何参数与校准

唯一的 URDF/Gazebo 机械参数入口是
`predictive_nav_description/urdf/robot.urdf.xacro` 顶部的 arguments。Gazebo
DiffDrive 从同一组 `wheel_radius`、`wheel_separation` arguments 获取运动学值。

| 分类 | 参数 | 当前值 | 说明 |
|---|---|---:|---|
| 硬件规格 | `base_radius` | 0.110 m | 220 mm 铝合金圆形底盘半径 |
| 硬件规格 | `wheel_radius` | 0.0325 m | 65 mm 名义轮径的一半；仍需以实际滚动半径标定 |
| 临时设计值 | `wheel_separation` | 0.220 m | 仅为可运行仿真，绝不是实测轮距 |
| 临时设计值 | Nav2 `robot_radius` | 0.135 m | 当前设计轮外缘半径 0.1235 m 加 11.5 mm 仿真余量 |
| 待实测 | LiDAR / IMU pose | 见 Xacro args | 都是相对 `base_link` 的 `x/y/z/rpy` |

真机装好后，测量左右驱动轮接地点的中心线距离并填写 `wheel_separation`；用直线里程计标定实际
`wheel_radius`；测量 caster 球半径和其安装点；测量 LiDAR 与 IMU 的六自由度位姿。

最后测量在任意姿态下最外侧机械件到 `base_footprint` 原点的距离。将其加上选定安全余量，统一填入
local/global costmap 的 `robot_radius`，并让后续 MPPI `DynamicRiskCritic.robot_radius_m` 使用同一个值。
`inflation_radius` 是障碍物代价的安全策略，不能拿来替代机器人本体半径，也不应在做该几何校准时随意改变。
