# 启动验证清单

这个清单用于每次启动仿真后快速确认系统是否处在稳定状态。建议每次修改 URDF、launch、Gazebo 插件、bridge、TF、RViz 配置或控制节点后，都按这个顺序检查一遍。

## 1. 编译并加载环境

```bash
cd /home/aderfd/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrt_description mrt_simulation
source install/setup.bash
```

预期结果：
- `mrt_description` 编译通过。
- `mrt_simulation` 编译通过。
- 没有 Python 语法错误、xacro 展开错误或 install 报错。

## 2. 启动仿真

```bash
ros2 launch mrt_simulation office_service_mvp.launch.py
```

预期结果：
- Gazebo 正常打开办公室场景。
- RViz 正常打开。
- `robot_1` 和 `robot_2` 都被 spawn 到地图中。
- RViz 中能看到地图标记、机器人模型、TF、激光和 odom。

如果只想测试后台话题，可以临时关闭 RViz：

```bash
ros2 launch mrt_simulation office_service_mvp.launch.py use_rviz:=false
```

## 3. 检查核心话题

另开一个终端：

```bash
cd /home/aderfd/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list
```

重点确认以下话题存在：

```text
/clock
/tf
/tf_static
/robot_1/cmd_vel
/robot_1/cmd_vel_safe
/robot_1/odom
/robot_1/scan
/robot_1/joint_states
/robot_2/cmd_vel
/robot_2/cmd_vel_safe
/robot_2/odom
/robot_2/scan
/robot_2/joint_states
```

如果缺少 `/robot_1/scan` 或 `/robot_1/odom`：
- 优先检查 `ros_gz_bridge` 是否启动。
- 再检查 Gazebo 中机器人是否 spawn 成功。

如果缺少 `/robot_1/cmd_vel_safe`：
- 优先检查 `collision_guard.py` 是否被安装并启动。

## 4. 检查话题频率

```bash
timeout 5 ros2 topic hz /robot_1/scan
timeout 5 ros2 topic hz /robot_1/odom
timeout 5 ros2 topic hz /tf
```

预期结果：
- `/robot_1/scan` 应该稳定发布，当前 lidar 配置约为 `10 Hz`。
- `/robot_1/odom` 应该稳定发布，当前 Gazebo odometry publisher 配置约为 `50 Hz`。
- `/tf` 应该持续更新。

如果频率明显不稳定：
- 检查 Gazebo 是否卡顿。
- 检查 bridge 是否持续运行。
- 检查是否有多个节点重复发布同一段 TF。

## 5. 检查 TF 链路

```bash
ros2 topic echo --once /tf_static
ros2 topic echo --once /tf
```

重点确认 TF 链路大致符合：

```text
world
└── robot_1/odom
    └── robot_1/base_footprint
        └── robot_1/base_link
            └── robot_1/top_deck_link
                └── robot_1/lidar_link
```

也可以使用：

```bash
timeout 5 ros2 run tf2_ros tf2_echo robot_1/odom robot_1/base_footprint
timeout 5 ros2 run tf2_ros tf2_echo robot_1/base_link robot_1/lidar_link
```

预期结果：
- `robot_1/odom -> robot_1/base_footprint` 会随机器人移动变化。
- `robot_1/base_link -> robot_1/lidar_link` 应该是固定变换。
- 蓝色小车顶部激光的 `lidar_link` 应该在车体上方，而不是车头前方低位。

## 6. 检查防撞速度链路

发布前进速度：

```bash
ros2 topic pub /robot_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}" -r 10
```

另开一个终端观察安全速度：

```bash
ros2 topic echo /robot_1/cmd_vel_safe
```

预期结果：
- 空旷位置时，`/robot_1/cmd_vel_safe` 的 `linear.x` 接近输入速度。
- 前方障碍进入 `slow_distance` 后，`linear.x` 逐渐变小。
- 前方障碍小于等于 `stop_distance` 后，`linear.x` 变成 `0.0`。

当前防撞参数位于 `src/mrt_simulation/launch/office_service_mvp.launch.py`：

```python
"stop_distance": 0.2,
"slow_distance": 0.5,
"front_sector_degrees": 46.0,
"rear_sector_degrees": 42.0,
```

如果小车停得太早：
- 适当调小 `slow_distance`。
- 适当调小 `stop_distance`，但不要小到车体已经碰墙才停。

如果小车容易撞墙：
- 适当调大 `stop_distance`。
- 适当调大 `slow_distance`。

当前 `cmd_timeout` 设置为 `0.0`，表示防撞节点不会因为键盘遥控没有重复发布而自动清零速度。使用 `teleop_twist_keyboard` 时，按 `i` 后小车会持续前进，按 `k` 或其他停止键后才停止。

## 7. 检查 RViz 和 Gazebo 是否一致

测试方法：

1. 在 Gazebo 中观察真实小车位置。
2. 在 RViz 中观察小车模型、激光点云和 odom 箭头。
3. 持续发布 `/robot_1/cmd_vel`，让小车接近墙体。

预期结果：
- Gazebo 中小车接近墙体后减速或停止。
- RViz 中小车和激光不再穿墙继续漂移。
- `/robot_1/cmd_vel_safe` 在接近障碍时被压低或清零。

如果 Gazebo 中停住但 RViz 中继续漂移：
- 优先检查 `/robot_1/odom` 和 `/tf`。
- 查看 `docs/debug_log.md` 中 “Gazebo 中小车撞墙停止，但 RViz 中小车继续前进” 的记录。

## 8. 检查蓝色小车顶部激光

蓝色小车 `robot_1` 的激光位置在 launch 中配置：

```python
lidar_x="-0.03",
lidar_y="0",
lidar_z="0.372",
```

检查目标：
- RViz 中 `/robot_1/scan` 应该能覆盖车体前后方向。
- 激光不应该被载荷箱明显遮挡。
- 防撞节点仍然能正确使用前方扇区。

如果后方仍然看不见：
- 先确认 RViz 的 LaserScan 显示固定坐标系是否正确。
- 再检查 `robot_1/base_link -> robot_1/lidar_link` 的 TF。
- 如果确实被车体或载荷遮挡，可以继续提高 `lidar_z`。

## 9. 结束测试

停止速度发布终端：

```bash
Ctrl+C
```

停止 launch：

```bash
Ctrl+C
```

确认没有残留进程：

```bash
pgrep -af "ros2|gz sim|parameter_bridge"
```

如果残留了旧的仿真或 bridge 进程，先清理后再重新测试，避免多个发布者干扰 TF 和话题。

## 10. 出问题时怎么记录

如果检查过程中发现新问题，把它追加到：

```text
docs/debug_log.md
```

推荐记录顺序：

```text
问题现象 -> 初步判断 -> 排查命令 -> 根因 -> 解决方法 -> 验证方法 -> 经验总结
```

## 11. robot_1 单车 SLAM 建图检查

启动 robot_1 SLAM：

```bash
cd /home/aderfd/PREDICTIVENAV2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrt_description mrt_simulation
source install/setup.bash

ros2 launch mrt_simulation robot_1_slam.launch.py
```

这个 launch 会做三件事：
- 启动办公室 Gazebo 仿真。
- 关闭普通 RViz 可视化中的 `world -> robot_1/odom` 静态 anchor。
- 启动 `slam_toolbox`，由 SLAM 发布 `map -> robot_1/odom`。
- 启动 SLAM 专用 RViz，固定坐标系为 `map`，显示 `/map`、`/robot_1/scan` 和 TF。

如果只想后台建图、不打开 RViz：

```bash
ros2 launch mrt_simulation robot_1_slam.launch.py use_slam_rviz:=false
```

确认 SLAM 相关话题：

```bash
ros2 topic list | grep -E "/map|/robot_1/scan|/robot_1/odom|/tf"
ros2 topic hz /robot_1/scan
ros2 topic hz /robot_1/odom
ros2 topic echo --once /map --field info
```

确认 TF：

```bash
timeout 5 ros2 run tf2_ros tf2_echo map robot_1/odom
timeout 5 ros2 run tf2_ros tf2_echo robot_1/odom robot_1/base_footprint
timeout 5 ros2 run tf2_ros tf2_echo robot_1/base_link robot_1/lidar_link
```

预期 TF 链路：

```text
map
└── robot_1/odom
    └── robot_1/base_footprint
        └── robot_1/base_link
            └── robot_1/top_deck_link
                └── robot_1/lidar_link
```

说明：
- `map -> robot_1/odom` 由 `slam_toolbox` 发布，会随着 scan matching 进行小幅修正。
- `robot_1/odom -> robot_1/base_footprint` 由 Gazebo odometry publisher 发布，反映仿真中小车的真实位姿。
- 如果 `map` 和 `robot_1/odom` 坐标轴逐渐分开，这是 SLAM 在修正 odom 累积误差，少量偏移是正常的。
- 如果小车在 RViz 中明显“闪回”或跳变，通常说明 SLAM 修正过大，应降低速度、减少打滑，并检查 `/robot_1/odom` 是否稳定。

手动遥控建图：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot_1/cmd_vel
```

常用按键：

```text
i    前进
,    后退
j    左转
l    右转
k    停止
u/o  前进并转弯
m/.  后退并转弯
q/z  整体加速/减速
w/x  只调线速度
e/c  只调角速度
```

建图建议：
- 慢速移动，优先沿墙和走廊边界走。
- 不要一直高速撞墙，观察 `/robot_1/cmd_vel_safe` 是否正常限速。
- 尽量闭环回到起点附近，方便 SLAM 做 loop closure。
- RViz 固定坐标系应为 `map`，并显示 `/map`。

保存地图：

```bash
mkdir -p src/mrt_simulation/maps
ros2 run nav2_map_server map_saver_cli \
  -f src/mrt_simulation/maps/office_service_mvp
```

保存成功后应生成：

```text
src/mrt_simulation/maps/office_service_mvp.yaml
src/mrt_simulation/maps/office_service_mvp.pgm
```

如果 `/map` 不出现：
- 检查 `slam_toolbox` 是否 active：

```bash
ros2 lifecycle get /slam_toolbox
```

- 检查 `/robot_1/scan` 是否有数据。
- 检查 `map -> robot_1/odom` 是否已经由 SLAM 发布。
- 确认启动 SLAM 时没有打开 `use_world_odom_anchors:=true`。
