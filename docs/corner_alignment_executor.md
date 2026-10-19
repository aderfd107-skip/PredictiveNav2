# 自动拐角预对齐执行器（Python V1）

`corner_alignment_executor.py` 是 Nav2 上层的**行为编排节点**，不是 DWB 或 MPPI 的替代品。
它接收一个最终目标，调用 Nav2 的 `/compute_path_to_pose` 请求全局路径，检测路径中航向变化明显的转角，
并在每个转角前后插入普通的 `NavigateToPose` 目标。

```text
最终目标
  → Nav2 全局路径
  → 检测明显转角
  → 转角前预对齐（已朝向下一段）
  → 转角后退出点
  → 最终目标
```

这样差速机器人可以在开阔区完成朝向变化，而不是贴着内侧凸角一边前进一边转弯。

## 安全边界

它不会发布 `/cmd_vel`，不会修改 `robot_radius`、costmap、DWB 权重、Gazebo 碰撞体或 MPPI 参数。
任一中间 Goal 失败或超时后，后续 Goal 都不会发送。

静态实验启动默认还包含独立的激光最终安全闸门：`/cmd_vel → /cmd_vel_safe → DiffDrive`。
它在任意方向的激光距离小于 `0.16 m` 时禁止平移，并在前/后方小于 `0.20 m` 时停止对应方向的平移；
圆形小车的纯原地转向仍允许。因此“发生实体碰撞后靠物理挤到目标”不再应被当作一次通过。

当前 V1 只依据全局路径几何识别转角，尚未验证中间点与连接路径的实时 costmap 间隙。因此：

- 先在当前静态实验场运行；
- 第一次先使用 `dry_run:=true`，只打印自动生成的中间点；
- 再在 Gazebo 中观察真实运行；
- 若路径临时被障碍物、重定位或重规划改变，停止测试而不是依赖旧的中间点。

后续 V2 才加入 live costmap footprint 碰撞检查、通道宽度评估和重规划处理。

## RViz 单击入口（推荐）

`predictive_dynamic_lab_nav.launch.py` 现在默认启动常驻的
`corner_alignment_goal_bridge.py`。RViz 工具栏中的 **2D Goal Pose** 已配置为发布到
`/corner_alignment/goal`，点击一次最终位置和朝向即可：

```text
RViz 2D Goal Pose
  → /corner_alignment/goal
  → corner_alignment_goal_bridge
  → 全局路径分析与自动中间航点
  → /navigate_to_pose
```

不要再使用旧的 **Nav2 Goal** 工具；它会绕过 bridge，直接把目标交给 Nav2，因而不会有自动预对齐。
bridge 支持**目标抢占**：任务执行期间再次点击，bridge 会保留最新点击、请求 Nav2 取消当前
`NavigateToPose` 小段，待该小段返回后，从机器人此刻的位置重新计算全局路径和自动中间航点。
这保证不会让旧任务和新任务同时向 Nav2 发送控制目标；连续点击时，以最后一次点击为准。

## 启动

终端 1 启动静态 Nav2 基线：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup predictive_dynamic_lab_nav.launch.py
```

等待 AMCL 与 Navigation 均为 `active`，再在 RViz 中执行一次 `2D Pose Estimate`。

终端 2 先进行只读预览。下面以开放区目标 `(-5.50, 4.60)` 为例：

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run predictive_nav_bringup corner_alignment_executor.py \
  --ros-args \
  -p goal_x:=-5.50 \
  -p goal_y:=4.60 \
  -p goal_yaw:=0.0 \
  -p dry_run:=true
```

它只会输出类似 `Detected turn ...` 和 `Stage ...`；不会使机器人移动。确认中间点合理后，把最后一行改为：

```bash
  -p dry_run:=false
```

## 可调的 V1 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `heading_lookahead_m` | `0.30` m | 计算转角前后方向时查看的路径距离。 |
| `turn_angle_threshold_rad` | `0.80` rad | 约 `46°`；更小会识别更多转角。 |
| `pre_alignment_distance_m` | `0.70` m | 在转角前多远的位置要求车头对齐下一段方向。 |
| `post_turn_distance_m` | `0.35` m | 转角后沿下一段前进多远再结束该转角阶段。 |
| `min_turn_separation_m` | `0.80` m | 相邻转角的最小合并距离。 |
| `max_turns` | `4` | 一次路径最多插入的转角数。 |

这些是路线行为参数，不是碰撞安全参数；不能以调小机器人半径替代它们。
