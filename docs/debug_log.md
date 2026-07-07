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

---

### 2026-07-07：teleop 键盘遥控三大问题 — 延迟、左右晃动、前进切左转先后退

#### 问题现象

使用 `teleop_twist_keyboard` 手动 SLAM 建图时，存在三个相互关联的问题：

1. **按键延迟**：按 `i` 前进后按 `j` 左转，机器人继续前进约 1 秒后才开始转向。反过来也一样。
2. **窄通道 / 转向时左右晃动**：在窄通道中行驶或转弯时，车身轻微左右摇摆。
3. **前进切左转先后退**：前进中按 `j` 左转，机器人会先向后倒退一小段距离，然后才左转。反过来（左转中按 `i` 前进）也会先反向转一点再前进。

之前已有其他人尝试修改加速度参数，但改完后不仅没解决，反而出现前进→后退→左转→右转的剧烈振荡，最终 Gazebo 直接崩溃启动不了。代码已回退到原始状态。

#### 根因分析（三次迭代才定位到真正根因）

**第一轮尝试（失败）—— 只改 collision_guard 非对称加减速：**

初步判断延迟来自于 `collision_guard.py` 的 `smooth_cmd()` 函数。它使用对称的 `limit_delta`：
- `linear_accel_limit=0.5` m/s²，从 0.5→0 减速需要 1 秒
- `angular_accel_limit=0.7` rad/s²，从 0→0.75 转向需要约 1 秒

修改方案：给 collision_guard 增加非对称加减速（`_limit_asymmetric`），减速用更高的 `_decel_limit`。参数调整为 `linear_accel=1.2, linear_decel=2.5, angular_accel=2.0, angular_decel=4.0`。

结果：**问题基本没改善，还残留**。延迟少了点但"先后退再转"和晃动依然存在。

**第二轮尝试（失败）—— 同时提高 Gazebo 限制：**

意识到控制链路有两层滤波器串联：

```
teleop → collision_guard（加减速限制，30Hz）
       → /cmd_vel_safe
       → Gazebo DiffDrive（又一套加减速限制，50Hz）
       → 实际运动
```

Gazebo DiffDrive 的限制（`max_linear_accel=0.8, min=-1.2, max_angular=0.9`）比 collision_guard 还保守。collision_guard 以为机器人已经减速了，Gazebo 实际还滞后——两个滤波器状态不一致，互相打架。

修改方案：把 Gazebo 加速度限制大幅提高到 ±4.0/±6.0，让 collision_guard 成为唯一权威。

结果：**晃得更厉害，先后退的问题也没解决**。两个滤波器虽然不再"限幅冲突"，但 collision_guard 的 `last_output` 跟踪的是自己发布的值而非 Gazebo 真实状态，状态不一致的本质问题并未消除。

**第三轮尝试（成功）—— 架构重构：**

最终认识到：**两个加速度滤波器串联在任何参数下都是错误架构**。正确做法是让 collision_guard 彻底放弃加速度限制，只做安全过滤。速度平滑完全交给 Gazebo DiffDrive 单层处理。

```
正确架构：
teleop → collision_guard（仅激光安全 + 透传）→ Gazebo DiffDrive（唯一的加减速）→ 运动
         单一职责：安全                      单一职责：物理平滑
```

在此架构下定位三个问题的真正根因：

| 问题 | 真正根因 | 
|------|---------|
| 延迟 | 原始 Gazebo `max_linear_accel=0.8, max_angular=0.9` 太保守 |
| 晃动 | `limit_axis` 只削减 linear.x 不削减 angular.z，靠墙时原地打转；EMA 滤波有 bug（`0.25×0.3 + 0.75×inf = inf`） |
| 先后退 | Gazebo DiffDrive 加减速不对称（加速 1.5 减速 3.0），减速太激进导致控制器 **overshoot 到负值** |

#### 最终解决方法

**修改了 3 个文件，另外还有一个意外修复：**

**1. `collision_guard.py` — 完全重写（去加速度限制，加安全机制）**

去掉的内容：
- `smooth_cmd()`、`_limit_asymmetric()`、`limit_delta()`、`limit_axis()` — 全部加速度限制逻辑
- `last_output`、`last_output_time` — 不再需要跟踪自己发布的速度

新增 / 修改的内容：
- 命令直接透传，不做任何加速度平滑
- **EMA 低通滤波**（`distance_smooth_alpha=0.25`）：解决激光单帧抖动导致速度波动，同时修复 inf→finite 过渡 bug
- **死区**（`<1e-4` 强制归零）：过滤浮点噪声
- **硬钳位**（输入≥0 则输出绝不<0）：最后的防线，杜绝任何反向速度
- **线速度和角速度同比例缩放**（`_apply_laser_safety`）：靠墙时 angular.z 和 linear.x 一起降，不再原地打转
- `cmd_timeout` 从 0.0 改为 0.3：松手 0.3 秒后自动停机

**2. `gazebo_plugins.xacro` — 对称加减速，适度提速**

```xml
<!-- 原始值 -->
<max_linear_acceleration>0.8</max_linear_acceleration>      <!-- 太慢 -->
<min_linear_acceleration>-1.2</min_linear_acceleration>     <!-- 不对称 -->
<max_angular_acceleration>0.9</max_angular_acceleration>     <!-- 太慢 -->
<min_angular_acceleration>-0.9</min_angular_acceleration>

<!-- 修改后 -->
<max_linear_acceleration>1.5</max_linear_acceleration>      <!-- ~2x 提速 -->
<min_linear_acceleration>-1.5</min_linear_acceleration>     <!-- 对称，不过冲 -->
<max_angular_acceleration>2.5</max_angular_acceleration>     <!-- ~3x 提速 -->
<min_angular_acceleration>-2.5</min_angular_acceleration>
```

关键：线加速度**对称 ±1.5**，消除控制器过冲（先后退的根因）。

**3. `office_service_mvp.launch.py` — 参数清理**

- 去掉废弃的 `linear_accel_limit`、`angular_accel_limit` 等参数（collision_guard 不再使用）
- 新增 `distance_smooth_alpha=0.25`
- `cmd_timeout` 从 0.0 改为 0.3

**4. 意外修复：`ParameterValue` 包装**

修改过程中发现 launch 启动报错 `Unable to parse the value of parameter robot_description as yaml`。因为 `robot_description` 是 URDF XML 字符串，包含 `<` 等 YAML 特殊字符，需要用 `ParameterValue(..., value_type=str)` 包装。添加了 `from launch_ros.parameter_descriptions import ParameterValue` 导入。

#### 响应时间对比

| 操作 | 原始 | 最终 |
|------|:---:|:---:|
| 0→0.5 m/s 加速 | 0.63s | **0.33s** |
| 0.5→0 减速 | 0.42s | **0.33s** |
| 0→0.75 rad/s 转向 | 0.83s | **0.30s** |
| 前进切左转 | 先后退再转 | 直接切换 |
| 窄通道行驶 | 左右晃动 | 平稳 |

#### 验证方法

```bash
cd /home/aderfd/ros2_multi_robot_task_planner
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrt_description mrt_simulation
source install/setup.bash
ros2 launch mrt_simulation robot_1_slam.launch.py

# 另开终端
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot_1/cmd_vel
```

测试要点：
- `i` 前进 → `j` 左转：应直接切换，无后退、无延迟
- 窄通道中 `i` 前进：不应左右晃动
- 靠近墙壁时：应自动减速直至停止

#### 经验总结

1. **两层同类型滤波器串联是反模式**。加速度限制只应在一个地方做。collision_guard 管安全，Gazebo 管物理，各司其职。
2. **状态估计必须来自真实数据源**。collision_guard 用 `last_output` 估计机器人速度，而 Gazebo 真实速度滞后——两个状态不一致导致各种奇怪行为。去掉加速度限制后不再需要估计，问题自然消失。
3. **非对称加减速本身没错，但要在正确的地方做**。Gazebo DiffDrive 支持 `max_linear_acceleration`（加速）和 `min_linear_acceleration`（减速，负值）两个独立参数，在 Gazebo 层就能实现非对称。但减速值不能太激进，否则控制器会 overshoot 到反向。
4. **线速度和角速度要联动**。靠墙减速时如果只降线速度不降角速度，机器人会原地打转，产生晃动。`ang_scale = min(front_scale, rear_scale)` 确保两者同步。
5. **EMA 滤波要考虑无穷大边界**。`0.25 × 0.3 + 0.75 × inf = inf`，传感器从"无障碍"变为"有障碍"时滤波值永远不更新。需要 `isfinite` 判断，inf→finite 直接跳变。
6. **URDF 字符串作为 ROS 参数需要 `ParameterValue` 包装**，否则 YAML 解析器会把 XML 标签当 YAML 语法报错。
