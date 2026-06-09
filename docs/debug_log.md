# 调试日志

这个文件用于记录项目中遇到的重要问题。每条记录都应该尽量包含：问题现象、初步猜测、验证命令、根因、解决方法、验证结果和经验总结。

## 记录模板

### YYYY-MM-DD：问题标题

#### 问题现象
- 观察到了什么异常？
- 是哪个工具中出现的问题？Gazebo、RViz、ROS topic、TF、launch 输出，还是其他地方？
- 这个问题是否稳定复现？

#### 初步判断
- 一开始怀疑是什么原因？
- 可能涉及哪个子系统？物理仿真、TF、odom、bridge、launch、URDF、SDF、RViz，还是控制逻辑？

#### 排查命令

```bash
# 写下排查过程中用到的命令。
```

#### 根因
- 最终确认真正的问题原因是什么？

#### 解决方法
- 修改了哪些文件？
- 修改后系统行为发生了什么变化？

#### 验证方法

```bash
# 写下用于验证修复结果的命令。
```

#### 经验总结
- 下次遇到类似问题时应该先想到什么？

---

### 2026-06-09：Gazebo 中小车撞墙停止，但 RViz 中小车继续前进

#### 问题现象
- Gazebo 中，小车撞到墙后物理上停止了。
- RViz 中，小车模型和激光扫描仍然继续向前移动。
- 最后激光甚至移动到了办公室地图范围之外。
- 这个现象在向 `/robot_1/cmd_vel` 持续发布前进速度时出现。

#### 初步判断
- Gazebo 中小车能被墙挡住，说明碰撞体和物理碰撞大概率是生效的。
- RViz 中小车继续移动，问题更可能出在 TF 或 odom 来源上。
- 重点怀疑链路是：`DiffDrive -> /robot_1/odom -> /tf -> RViz`。
- 小车底盘被墙挡住后，轮子可能还在继续转，导致差速驱动插件继续发布向前运动的 odom/TF。

#### 排查命令

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic info /robot_1/odom -v
ros2 topic info /tf -v
ros2 topic echo /tf
ros2 topic echo /robot_1/cmd_vel
ros2 topic echo /robot_1/scan
```

修复后也可以观察安全速度输出：

```bash
ros2 topic echo /robot_1/cmd_vel_safe
```

#### 根因
- Gazebo 的 `gz::sim::systems::DiffDrive` 插件会根据驱动状态发布 odom 和 TF。
- 小车撞墙时，Gazebo 物理系统让底盘停止，但持续输入的速度命令仍可能让轮子继续转动。
- RViz 使用的是 `robot_1/odom -> robot_1/base_footprint` 这段动态 TF，所以即使 Gazebo 中底盘已经停住，RViz 中的小车和激光仍会跟随 odom/TF 继续向前漂移。

#### 解决方法
- 在以下文件中增加差速驱动限速、限加速度、限 jerk，并调整轮子接触参数：
  - `src/mrt_description/urdf/gazebo_plugins.xacro`
- 将 Gazebo 差速驱动插件的输入话题从普通速度命令改为安全速度命令：
  - 原来：`/robot_1/cmd_vel`、`/robot_2/cmd_vel`
  - 现在：`/robot_1/cmd_vel_safe`、`/robot_2/cmd_vel_safe`
- 新增基于激光的防撞速度过滤节点：
  - `src/mrt_simulation/scripts/collision_guard.py`
- 在 launch 文件中为每台车启动防撞节点：
  - `src/mrt_simulation/launch/office_service_mvp.launch.py`
- 更新安装配置和依赖：
  - `src/mrt_simulation/CMakeLists.txt`
  - `src/mrt_simulation/package.xml`

现在的速度命令链路是：

```text
/robot_1/cmd_vel -> collision_guard -> /robot_1/cmd_vel_safe -> Gazebo DiffDrive
```

#### 防撞节点参数

参数配置位置：`src/mrt_simulation/launch/office_service_mvp.launch.py`

```python
"stop_distance": 0.48,
"slow_distance": 0.85,
"front_sector_degrees": 46.0,
"rear_sector_degrees": 42.0,
"publish_rate": 30.0,
"cmd_timeout": 0.35,
```

参数含义：
- `slow_distance`：当前方障碍物进入这个距离以内时，开始降低前进速度。
- `stop_distance`：当前方障碍物小于等于这个距离时，强制将前进速度置为 0。
- `front_sector_degrees`：前进时只检查车头正前方这个角度范围内的激光。
- `rear_sector_degrees`：倒车时检查车尾方向这个角度范围内的激光。
- `cmd_timeout`：如果输入速度命令超时，就主动发布 0 速度。

#### 验证方法

编译并 source：

```bash
cd /home/aderfd/ros2_multi_robot_task_planner
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrt_description mrt_simulation
source install/setup.bash
```

启动仿真：

```bash
ros2 launch mrt_simulation office_service_mvp.launch.py
```

发布前进速度：

```bash
ros2 topic pub /robot_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}" -r 10
```

观察防撞节点输出：

```bash
ros2 topic echo /robot_1/cmd_vel_safe
```

预期结果：
- 当前方激光检测到墙体靠近时，小车开始减速。
- 距离过近时，小车停止继续向墙推进。
- RViz 中的小车和激光不再继续穿墙漂移。

#### 经验总结
- RViz 中显示异常，不一定是 RViz 本身的问题，很多时候根源在 TF 或 odom。
- 当 Gazebo 和 RViz 显示不一致时，应优先比较物理模型真实状态、`/odom` 和 `/tf`。
- 差速驱动仿真中，如果轮子撞墙后继续空转，轮式 odom 可能继续积分并产生漂移。
- 在公开的 `/cmd_vel` 和 Gazebo 实际驱动话题之间加一层安全过滤，可以避免很多不真实的仿真行为。
- 记录完整的话题链路非常重要，后面再遇到类似问题会排查得更快。
