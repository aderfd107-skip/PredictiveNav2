# Predictive Dynamic Lab：静态地图基线

世界文件 `predictive_dynamic_lab.sdf` 是动态预测实验的静态底座，尺寸为 `20 m × 16 m`，
世界名为 `predictive_dynamic_lab`。它没有动态 actor；所有墙均为 `0.12 m` 厚、`1.2 m` 高。
默认起点为 `(-4.5, -6.8, 0)`、朝向 `+X`，位于下方中央的 staging 区，而非任何实验通道内。

| 区域 | 近似位置（m） | 静态用途 | 后续单 actor 实验 |
|---|---|---|---|
| 开放区 | x=-9..-3, y=2..8 | 宽阔绕行、双 actor 压力测试 | 横向或对角横穿 |
| 十字横穿区 | x=-2.5..3.5, y=2..8 | 1.10 m 的十字交汇 | actor 纵向，机器人横向通过中心 |
| 迎面走廊 | x=4..9.7, y=5.45..6.55 | 5 m 直走廊，净宽 1.10 m | 两端反向接近 |
| 盲区 L 转角 | x=-8.8..-4.5, y=-7.5..-2.2 | 内角遮挡 LiDAR | actor 从转角外横支路进入 |
| T 字路口 | x=-4.5..3.5, y=-7.5..-2.45 | 1.00 m 进场和横向支路 | actor 横穿，机器人从南侧转入 |
| 90° 转角 | x=1..5, y=-6.45..-2.6 | 0.90 m L 形通道 | actor 从不可见支路出现 |
| 窄通道 | x=7.45..8.15, y=-7.5..-2.1 | 当前净宽约 0.70 m | 以后按最终半径重做会车尺寸 |

> 注意：SDF 两道墙的中心线相距 `0.82 m`，扣除两侧各 `0.06 m` 的墙厚后，当前净宽为
> `0.70 m`。真机半径、actor 半径和安全余量确定后，仍应以最终净宽而非墙中心距重设会车通道。

## 静态验收顺序

```bash
cd ~/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_simulation predictive_dynamic_lab_static.launch.py
```

先检查模型没有穿墙、所有区域在 LiDAR 中可见、L 转角确实造成遮挡、机器人可以通过预期通道。
此阶段只做手动驾驶，不启动任何 actor。

确认静态几何后，再启动 mapping：

```bash
ros2 launch predictive_nav_simulation mapping.launch.py
```

手动覆盖所有可达区域并保存 map；保存后的 PGM/YAML 才能成为后续 AMCL、DWB、原版 MPPI 和
Predictive MPPI 共用的已知静态地图。动态 actor 必须在保存 map 后的独立 trial launch 中加入，且
每次只启用一个确定性路线。

## 已知地图静态 Nav2 基线

地图由 SLAM 新保存到 `src/predictive_nav_simulation/maps/` 后，先重新构建一次。
Nav2 launch 从安装空间读取包资源；若跳过此步，`map_server` 会找不到新 YAML，随后 `map` 和
`map -> odom` 都不会出现：

```bash
cd ~/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_simulation predictive_nav_bringup --symlink-install
source install/setup.bash
```

然后使用下面的独立入口启动 AMCL + Nav2 DWB。默认地图为
`predictive_dynamic_lab_truth.yaml`：它由同一份 SDF 静态 collision box 在
`0.025 m/cell` 分辨率下生成，保留物理 `0.44 m` 窄出口而不继承 SLAM 栅格误差。
原始 `predictive_dynamic_lab.yaml` 会被保留，仍可通过 launch 的 `map:=...` 参数
选择。Gazebo world 名称和起点 `(-4.5, -6.8, 0)` 保持不变，且明确禁用动态 actor：

```bash
cd ~/PREDICTIVENAV2
source install/setup.bash
ros2 launch predictive_nav_bringup predictive_dynamic_lab_nav.launch.py
```

若 SDF 墙体发生修改，重新生成真值地图后再构建：

```bash
python3 src/predictive_nav_simulation/scripts/generate_truth_map.py \
  --world src/predictive_nav_simulation/worlds/predictive_dynamic_lab.sdf \
  --output src/predictive_nav_simulation/maps/predictive_dynamic_lab_truth.pgm
colcon build --packages-select predictive_nav_simulation predictive_nav_bringup --symlink-install
```

窄通道基线的 local/global costmap 也使用 `0.025 m` 分辨率；这是为了避免更精细
的静态地图在控制器中被重新采样成 `0.05 m`。机器人半径仍为 `0.135 m`，不以缩小
footprint 的方式取得通过率。该分辨率会增加计算量，必须在启动后检查 controller loop rate。

在 RViz 中等待 Nav2 服务器就绪后，先确认 `/scan` 端点与黑色静态墙重合，再用 **Nav2 Goal**
依次测试开放区、十字区、T 字口、90°转角和窄通道。不要在此阶段启用 actor；这组结果就是后续
动态避障实验的静态对照组。

该 Nav2 入口将 Gazebo DiffDrive 直接映射到 Nav2 的 `/cmd_vel`，并关闭只用于手动遥控的
`scan_safety_guard`。障碍物安全由 Nav2 的激光 obstacle layer、`robot_radius` 和 inflation layer
统一处理；这避免 guard 在靠墙转向时把角速度归零，也保证后续 DWB、MPPI 和 Predictive MPPI 使用
相同的控制链路。

## 已记录：小转角的单 Goal 暂停 / 恢复

**状态：先使用 SDF 真值地图复现；不修改底盘几何或 DWB 运动参数。**

在无动态 actor、AMCL 和 Nav2 均为 active 的静态基线中，曾观察到以下现象：

- 对跨越小转角的远距离单个 Nav2 Goal，机器人有时会在转角前，或转角后的开阔区域短暂停住；
- `controller_server` 会报告 `Failed to make progress`，Nav2 随后清空 local costmap 并执行恢复；
- 将同一路线拆成“到转角附近”与“穿过转角”两个 Goal 后，机器人可正常通过；
- 窄通道已通过分段 Goal 验证可以从南侧入口进入，因而不能把上述现象直接归因于地图不可通行。

这说明当前 220 mm 圆形机器人模型和地图几何仍是可用的，但默认 DWB 局部控制参数尚未针对本实验室
的连续小转角完成静态回归。该项必须在加入动态 actor 前处理，否则会把静态局部控制停顿误记为动态
预测或 DynamicRiskCritic 的效果。

后续处理顺序：先消除地图中意外形成的过窄出口，再固定一个可复现的起点、终点和朝向；显示
local/global costmap 与 DWB 局部轨迹；确认暂停时是过度贴合全局路径、靠墙代价过高，还是原地转向；
最后只调整相应 DWB 参数并记录前后 recovery、完成时间和最小墙距。不要仅通过增大 progress checker
的等待时间掩盖问题。

2026-09-13 记录：上方小口袋出口的物理净开口为 `0.44 m`。原始保存 SLAM 地图在这里约为
`0.40 m`（`0.05 m/cell`），会放大栅格误差。该出口保留为**窄通道专项测试**，Gazebo 世界不改宽；
默认改用与 SDF collision box 对齐的 `0.025 m/cell` 真值地图。原始 SLAM 地图仍保留，用于单独
评价建图误差，而不以缩小机器人半径或强迫控制器前进取得通过率。

### 回归路线 R1：上方小转角

用户在 RViz 中确认当前上方小转角位置后，从正在执行的 `/plan` 捕获到如下**近似 map 坐标**。目标朝向
约为 `-0.021 rad`，实际首轮诊断可保持 `0 rad`。

| 项目 | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| 观察到的起点 R1-S | -2.153 | 5.916 | 0.0 |
| 终点 R1-G | 2.307 | 6.339 | -0.021 |

> 这些是 AMCL 的 map 估计坐标，**不是已验证的 Gazebo 无碰撞出生坐标**。不得直接把它们传给
> `spawn_x/spawn_y`；那样会因 map--odom 估计偏差把机器人生成到墙边。R1 自动回归前，应先从
> `(-4.5, -6.8, 0)` 的蓝色出生垫启动，手动驶到一个无碰撞的转角前置点，并记录当时的 `/odom` 与
> `/amcl_pose`，再决定可重放的出生位姿。

当前 Gazebo DiffDrive 在重新生成模型时会重置 / 重建 odom 参考，运行中的 `/odom` 与 `/amcl_pose`
成对记录不能直接用作下一次启动参数；仍应从默认蓝色出生垫启动，并通过 RViz 的 `2D Pose Estimate`
完成一次可见的初始对齐。

在已验证的无碰撞前置点上，另开终端发送固定终点，便可检查“单个长 Goal 跨越小转角时的暂停”问题：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.307, y: 6.339, z: 0.0}, orientation: {z: -0.0104, w: 0.99995}}}}"
```

右下窄通道入口仍作为独立的 R2 场景记录，不能与 R1 的小转角暂停混为一个问题。R1/R2 都属于
“能否通过”的压力测试；若真机标定后的机器人外接圆、安全余量和实际净宽不满足条件，正确行为是
绕行或报告不可达，而不是降低 `robot_radius` 强行通过。

### R1 调优状态

`predictive_dynamic_lab_nav.launch.py` 当前使用 `nav2_dwb.yaml` 中的**原始 DWB 基线**，不再对任何
`PathAlign`、`GoalAlign`、`PathDist`、`GoalDist` 或 `sim_time` 参数做实验覆盖。先确认此前可通过的
普通路线恢复正常，再为 R1 单独建立可重复的测试与参数实验；不在全地图导航仍不稳定时继续调参。

### DWB 只读诊断

`FollowPath.debug_trajectory_details: true` 仅发布候选轨迹评分，不改变速度命令。让机器人复现卡住状态
后，在另一终端运行：

```bash
ros2 run predictive_nav_bringup dwb_evaluation_monitor.py \
  --ros-args -p topic:=/evaluation
```

它会读取一次 `/evaluation`，打印最佳候选的 `x/theta` 速度及每个 critic 的分数，然后
自动退出。输出可用于区分“所有前进候选被碰撞检查否决”和“安全候选被路径/目标评分压低”，在此之前
不得改变机器人半径、地图或强迫更快转向。

2026-09-13 运行时诊断记录：卡住时 `/cmd_vel` 由 `controller_server` 发布、Gazebo bridge 已订阅，
但命令为 `linear.x = 0`、`angular.z ≈ 0.0304 rad/s`。因此不是 Gazebo、TF 或桥接中断，而是 DWB
选择了无实质进展的微小原地转动。该现象用于后续诊断“为什么没有更好的安全轨迹”；不通过提高
`min_speed_theta` 强迫更快转向，也不改变机器人几何、地图或安全半径。
