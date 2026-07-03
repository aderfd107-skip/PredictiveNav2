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

---

### 2026-06-09：SLAM 建图时 map 和 odom 偏离过大，小车在 RViz 中闪回

#### 问题现象
- 运行 `robot_1_slam.launch.py` 手动建图时，RViz 中 `map` 坐标轴和 `robot_1/odom` 坐标轴距离越来越远。
- 小车在 RViz 中会出现“向前走一段，然后突然闪回之前位置”的现象。
- Gazebo 中模型位置相对稳定，但 RViz 中的机器人、激光和地图匹配结果会发生明显跳变。

#### 初步判断
- `map` 和 `odom` 有偏移本身是正常的，因为 SLAM 会发布 `map -> robot_1/odom` 来修正里程计误差。
- 但是偏移过大并伴随闪回，说明里程计和激光匹配结果差异太大。
- 重点怀疑：
  - 轮式 odom 受打滑影响产生漂移；
  - 狭窄走廊中激光匹配约束较强，SLAM 会突然修正位姿；
  - 轮速里程计和 Gazebo 中模型真实位姿不一致。

#### 排查命令

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic info /robot_1/odom -v
ros2 topic info /tf -v
ros2 topic echo --once /map --field info
ros2 lifecycle get /slam_toolbox
ros2 run tf2_ros tf2_echo map robot_1/odom
ros2 run tf2_ros tf2_echo robot_1/odom robot_1/base_footprint
```

#### 根因
- 原先主 `/robot_1/odom` 和 `/tf` 主要来自 Gazebo DiffDrive 的轮式里程计。
- 仿真中小车转向、接触墙体或在狭窄空间运动时，轮子可能发生打滑或非理想接触。
- 轮式 odom 积分出来的位置和 Gazebo 中模型真实位置逐渐不一致。
- SLAM 根据激光扫描重新匹配地图时，会通过 `map -> odom` 做较大修正，于是 RViz 中表现为机器人位置突然跳变。

#### 解决方法
- 修改 `src/mrt_description/urdf/gazebo_plugins.xacro`：
  - DiffDrive 不再发布主 `/robot_1/odom` 和 `/tf`。
  - DiffDrive 的里程计改为调试用的 `robot_1/wheel_odom` 和 `robot_1/wheel_tf`。
  - 新增 Gazebo `OdometryPublisher`，用模型真实位姿发布主 `robot_1/odom` 和 `robot_1/tf`。
  - 降低差速驱动的角速度、角加速度和角 jerk，减少急转时的物理抖动。
- 修改 `src/mrt_simulation/config/robot_1_slam.yaml`：
  - 使用 `robot_1/odom`、`robot_1/base_footprint` 和 `/robot_1/scan`。
  - 关闭 `do_loop_closing`，避免早期建图时因为回环检测造成较大的位姿跳变。
  - 调整扫描更新频率和最小运动阈值，让建图过程更稳定。
- 修改 `src/mrt_simulation/launch/robot_1_slam.launch.py`：
  - SLAM 模式下关闭普通 RViz，单独启动 `robot_1_slam.rviz`。
  - 关闭 `world -> robot_1/odom` 静态锚点，避免和 SLAM 的 `map -> robot_1/odom` 变换职责混在一起。

#### 验证方法

```bash
cd /home/aderfd/ros2_multi_robot_task_planner
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrt_description mrt_simulation
source install/setup.bash

ros2 launch mrt_simulation robot_1_slam.launch.py
```

预期结果：
- RViz 中机器人不再频繁闪回。
- `map -> robot_1/odom` 仍可能有小幅修正，但不应持续拉得很远。
- 手动建图时地图边界和激光扫描更稳定。

#### 经验总结
- SLAM 中 `map` 和 `odom` 分离是正常设计，不应该强行让它们永远重合。
- 如果 `map -> odom` 修正很大，并且机器人在 RViz 中闪回，应优先检查 odom 的发布源和 TF 是否重复。
- 仿真中做 SLAM 时，轮式 odom 不一定是最稳定的主 odom；如果目标是先验证导航和建图流程，可以用 Gazebo 真实位姿 odom 降低干扰。
- 狭窄场景里建图应降低速度和角速度，避免激光匹配被急转、打滑和墙面近距离扫描放大误差。
