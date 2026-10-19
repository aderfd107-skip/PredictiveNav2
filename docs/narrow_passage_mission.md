# 窄通道顺序航点测试

`narrow_passage_mission.py` 用多个独立的 `NavigateToPose` 目标自动复现手动分段通过 `0.44 m` 出口的过程。它不修改 DWB、代价地图、机器人半径或 Gazebo 碰撞模型。

每一段必须成功，才会下发下一段；任一段被拒绝、失败或在 90 秒内没有完成，脚本会取消当前目标并停止。因此它不会为“完成路线”而强迫机器人向前顶墙。

先启动静态基线：

```bash
source /opt/ros/jazzy/setup.bash
source ~/PredictiveNav2/install/setup.bash
ros2 launch predictive_nav_bringup predictive_dynamic_lab_nav.launch.py
```

待 Nav2 激活和初始定位稳定后，在另一个终端执行北向穿越：

```bash
source /opt/ros/jazzy/setup.bash
source ~/PREDICTIVENAV2/install/setup.bash
ros2 run predictive_nav_bringup narrow_passage_mission.py \
  --ros-args -p route:=northbound_outlet
```

南向穿越使用：

```bash
ros2 run predictive_nav_bringup narrow_passage_mission.py \
  --ros-args -p route:=southbound_outlet
```

两条路线的通道圆心线均为 `x=-2.72 m`。这由两侧实体墙面 `x=-2.94 m` 与 `x=-2.50 m` 的中点得到；机器人仍使用 `robot_radius=0.135 m`，故不会把安全距离伪装成通行能力。

## 为什么增加“预对齐”航点

每个方向现在都有 5 段，而非原来的 4 段：

```text
开阔区接近 → 远离凸角的预对齐 → 槽口前对齐 → 穿越槽口 → 离开出口
```

北向预对齐点为 `(-2.72, 4.55, +pi/2)`，距右侧凸角的纵向距离约 `0.70 m`；
南向预对齐点为 `(-2.72, 6.07, -pi/2)`，同样保留约 `0.70 m`。因此机器人应在开阔侧完成朝向改变，之后沿槽口中心线直行，而不是在右侧凸角旁边一边前进一边转弯。

这只是**测试路线的几何修正**，不表示窄通道已经被“放宽”：槽口仍为 `0.44 m`，机器人半径仍为 `0.135 m`，代价地图和 DWB 参数完全不变。若预对齐后的直线穿越仍发生碰撞，就应继续检查局部 footprint、控制轨迹和实际 Gazebo 碰撞，而不是继续移动航点。

该脚本旨在测量“带路线语义航点的窄通道能力”。它不能替代自由单目标导航测试：后者仍应单独记录是否卡住、控制器评分与恢复次数。
