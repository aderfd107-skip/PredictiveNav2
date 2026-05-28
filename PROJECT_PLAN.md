# 基于 ROS 2 的室内多机器人巡检任务分配与无冲突路径规划仿真平台

## 0. 文档目的

这份文档是新项目的最终计划草案，用来在正式开工前明确项目方向、系统边界、技术路线、阶段目标和评测方式。

后续需要基于以下渠道继续校准和修改：

- ROS 2 Jazzy 官方文档
- Nav2 官方文档和 GitHub
- Gazebo Harmonic 官方文档
- Open-RMF 文档和源码
- ROS Discourse
- Moving AI MAPF benchmark
- 真实机器人公司招聘要求和技术博客

这份计划不是最终不可改的规格书，而是第一版“项目总纲”。之后每次查资料、做实验、发现实现难点，都应该反向更新本文档。

## 1. 项目一句话定义

基于 ROS 2 Jazzy 构建一个室内多机器人巡检仿真平台，在已知办公楼地图中完成多机器人任务分配、无冲突路径规划、Nav2 执行集成、失败恢复和批量评测。

英文描述：

```text
A ROS 2 Jazzy based multi-robot indoor inspection simulation platform
with task allocation, conflict-free path planning, Nav2 execution,
failure recovery, and benchmark evaluation.
```

## 2. 项目背景

已有项目是一个 ROS 2 Humble 单机器人扫地机器人仿真项目，核心能力包括：

- Gazebo 仿真
- SLAM 建图
- 已知地图导航
- 覆盖路径生成
- waypoint 下发
- 清扫任务状态机
- 覆盖率统计
- waypoint 失败恢复

这个项目已经能证明：

- 能搭建 ROS 2 仿真链路
- 能使用 Nav2 完成底层导航
- 能写高层路径规划逻辑
- 能写任务状态机
- 能把多个 ROS 包集成成完整流程

但它的上限也比较明显：

- 单机器人
- 单任务流
- 主题是清扫，扩展到多机器人容易不自然
- 没有多机器人冲突规划
- 没有系统性的评测平台
- 算法对比和实验数据不足
- C++ 工程能力展示不足

因此新项目不继续在原项目上堆功能，而是以 ROS 2 Jazzy 重新设计一个更强的多机器人巡检系统。

## 3. 项目定位

项目定位为：

```text
偏系统软件的机器人多机器人调度与规划仿真平台
```

不是：

- 纯算法论文复现
- 单纯多机器人 launch demo
- 只会调用 Nav2 的应用层拼装
- 炫技式大而全平台
- 真机项目
- 云端管理平台

是：

- 有明确室内巡检场景
- 有清晰系统架构
- 有任务分配逻辑
- 有无冲突路径规划逻辑
- 有 Nav2 执行集成
- 有失败恢复
- 有批量实验
- 有指标统计和图表
- 有部分 C++ 核心模块
- 有完整文档和可展示 demo

## 4. 目标岗位方向

项目主要面向：

- 机器人系统软件工程师
- 机器人平台开发工程师
- 机器人导航系统开发工程师
- 机器人仿真测试平台工程师

不主要面向：

- 纯感知算法岗
- 纯控制算法岗
- 纯前端岗
- 纯后端业务岗
- 纯数学优化研究岗

项目在简历中的重点应该是：

- ROS 2 多机器人系统集成
- Nav2 多机器人执行
- 任务调度状态机
- 无冲突路径规划
- 仿真评测平台
- C++ ROS 节点开发
- 工程化文档和实验结果

## 5. 核心问题拆解

这个项目本质上有三个核心问题。

### 5.1 任务分配

回答：

```text
哪个机器人去做哪个任务？
任务什么时候开始？
任务失败后由谁接手？
突发任务如何插入？
```

输入：

- 机器人当前位置
- 机器人状态
- 当前任务队列
- 任务优先级
- 任务目标点
- 已分配任务
- 机器人预计到达时间

输出：

- 任务到机器人的分配结果
- 任务执行顺序
- 任务重分配结果

### 5.2 无冲突路径规划

回答：

```text
多台机器人如何在同一张地图上移动，同时避免时空冲突？
```

需要处理：

- 同一时刻占用同一位置
- 两台机器人对向交换位置
- 窄通道会车
- 共享走廊等待
- 动态任务插入后的重规划
- 某台机器人失败后的局部重规划

### 5.3 仿真评测

回答：

```text
这个系统到底比普通方案好在哪里？
不同算法在不同场景下有什么差异？
```

需要输出：

- makespan
- 总路径长度
- 平均等待时间
- 任务成功率
- 冲突次数
- 重规划次数
- 机器人利用率
- 不同算法对比图

## 6. 调度、规划、导航的边界

本项目必须明确三者的职责。

### 6.1 调度

调度负责：

- 管理任务队列
- 给机器人分配任务
- 判断任务优先级
- 插入突发任务
- 处理失败重分配
- 维护任务生命周期

调度不负责：

- 机器人局部避障
- 轮速控制
- 激光建图
- AMCL 定位

### 6.2 路径规划

路径规划负责：

- 在地图上寻找从起点到目标点的路径
- 生成多机器人无冲突时空路径
- 判断共享通道占用
- 安排等待动作
- 提供高层 waypoint

路径规划不负责：

- 底盘控制
- 实时避障控制
- 传感器融合

### 6.3 导航

导航负责：

- 接收目标点
- 局部规划
- 避障
- 控制机器人运动
- 到达目标后反馈结果

本项目中底层导航主要依赖 Nav2。自己的核心贡献不在重写 Nav2，而在 Nav2 上层的多机器人调度、无冲突规划和系统集成。

## 7. 应用场景

推荐场景：

```text
室内办公楼 / 园区楼层巡检
```

典型地图元素：

- 长走廊
- 多个办公室
- 会议室
- 设备间
- 前台区域
- 窄通道
- T 字路口
- 十字路口
- 共享充电区

典型任务：

- 固定巡检点访问
- 区域巡检
- 设备间紧急检查
- 会议室状态检查
- 突发任务插入
- 机器人失败后的任务接管

第一版只做 2D 平面巡检，不做多楼层和电梯。

## 8. 最终系统形态

最终版本应该可以展示以下能力：

- 3 台机器人在同一办公楼地图中运行
- 地图中有 15 到 30 个巡检点
- 系统可以加载巡检任务配置
- 调度器可以分配任务
- 规划器可以生成无冲突路径
- 机器人可以通过 Nav2 执行任务
- 窄通道中机器人会等待和让行
- 插入突发任务后系统可以重新分配
- 某台机器人失败后任务可以由其他机器人接管
- 系统可以记录每次实验数据
- 可以批量运行不同算法组合
- 可以生成实验图表
- 核心模块中至少有一部分使用 C++

## 9. 技术基线

计划技术栈：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2
- RViz2
- Python 3
- C++17 或 C++20
- CMake / ament
- YAML
- CSV / JSON
- matplotlib 或其他 Python 绘图库

语言分工建议：

```text
Python：
- 离线算法原型
- 场景生成
- 实验脚本
- 批量评测
- 数据统计
- 绘图
- 早期 ROS 节点

C++：
- 核心规划器
- 核心任务管理器
- Nav2 action 执行层
- 冲突检测模块
- 性能敏感模块
```

项目后期目标比例：

```text
Python 30%：实验、工具、评测
C++ 70%：ROS 核心节点和规划执行
```

## 10. 总体架构

建议系统架构：

```text
scenario yaml
  |
  v
scenario loader
  |
  v
task manager <------------------------------+
  |                                         |
  v                                         |
task allocator                              |
  |                                         |
  v                                         |
multi-robot planner                         |
  |                                         |
  v                                         |
execution manager                           |
  |                                         |
  v                                         |
Nav2 action clients for each robot          |
  |                                         |
  v                                         |
Gazebo simulation                           |
  |                                         |
  v                                         |
robot state monitor ------------------------+
  |
  v
metrics logger
  |
  v
csv / json / plots
```

## 11. 推荐仓库结构

```text
multi_robot_inspection_project/
├── PROJECT_PLAN.md
├── README.md
├── docs/
│   ├── system_design.md
│   ├── ros_interfaces.md
│   ├── algorithms.md
│   ├── experiments.md
│   ├── failure_recovery.md
│   ├── nav2_integration.md
│   ├── gazebo_setup.md
│   └── openrmf_notes.md
├── maps/
│   ├── office_small.yaml
│   ├── office_small.pgm
│   ├── office_corridor.yaml
│   └── office_multi_room.yaml
├── scenarios/
│   ├── mvp_2robots_8tasks.yaml
│   ├── corridor_conflict.yaml
│   ├── narrow_passage.yaml
│   ├── dynamic_task.yaml
│   └── robot_failure.yaml
├── experiments/
│   ├── configs/
│   ├── results/
│   └── plots/
├── scripts/
│   ├── run_offline_experiment.py
│   ├── run_ros_experiment.py
│   ├── plot_results.py
│   └── generate_scenario.py
└── src/
    ├── inspection_msgs/
    ├── inspection_description/
    ├── inspection_simulation/
    ├── inspection_bringup/
    ├── inspection_core/
    ├── inspection_task_manager/
    ├── inspection_allocator/
    ├── inspection_planner/
    ├── inspection_executor/
    ├── inspection_evaluator/
    └── inspection_visualization/
```

## 12. ROS 包职责

### 12.1 `inspection_msgs`

定义项目自定义接口。

可能包含：

- `InspectionTask.msg`
- `InspectionTaskArray.msg`
- `RobotState.msg`
- `RobotStateArray.msg`
- `TaskAssignment.msg`
- `TaskAssignmentArray.msg`
- `TimedPath.msg`
- `TimedWaypoint.msg`
- `MissionMetrics.msg`
- `TaskStatus.msg`

后续也可以定义 service 和 action：

- `SubmitTask.srv`
- `CancelTask.srv`
- `Replan.srv`
- `ExecuteInspectionTask.action`

### 12.2 `inspection_description`

机器人模型包。

职责：

- 机器人 URDF / Xacro
- LiDAR
- IMU
- 差速底盘
- 多机器人命名空间支持

第一版可以复用旧项目的模型思想，但不要直接照搬名字和结构。新项目要服务于多机器人巡检。

### 12.3 `inspection_simulation`

仿真场景包。

职责：

- Gazebo world
- 办公楼地图
- 障碍物
- 巡检点 marker
- 多机器人 spawn

建议至少准备 4 类场景：

- open_area：开阔区域
- corridor：走廊
- narrow_passage：窄通道
- multi_room：多房间

### 12.4 `inspection_bringup`

启动入口包。

职责：

- 启动 Gazebo
- 启动 robot_state_publisher
- 启动 Nav2
- 启动 AMCL
- 启动调度、规划、执行节点
- 启动 RViz

建议入口：

- `single_robot_nav.launch.py`
- `multi_robot_nav.launch.py`
- `inspection_mvp.launch.py`
- `inspection_eval.launch.py`

### 12.5 `inspection_core`

纯算法和数据结构库。

职责：

- 地图加载
- 栅格图结构
- 任务结构
- 机器人结构
- A*
- reservation table
- 冲突检测
- 离线仿真
- 指标计算

这个包尽量不要强依赖 ROS，便于单元测试和离线实验。

### 12.6 `inspection_task_manager`

任务管理节点。

职责：

- 维护任务队列
- 维护任务状态机
- 接收新任务
- 通知分配器
- 处理任务完成和失败
- 触发重分配

### 12.7 `inspection_allocator`

任务分配节点。

职责：

- 根据机器人状态和任务状态计算分配
- 支持不同分配策略
- 输出任务分配结果

策略：

- nearest
- earliest_finish_time
- auction
- hungarian

### 12.8 `inspection_planner`

多机器人路径规划节点。

职责：

- 接收任务分配结果
- 读取地图
- 获取机器人当前位置
- 计算多机器人无冲突路径
- 输出 timed path

算法：

- independent A*
- prioritized planning
- CBS

### 12.9 `inspection_executor`

执行节点。

职责：

- 将 timed path 转换为 Nav2 目标
- 管理每台机器人 Nav2 action client
- 监听执行结果
- 向 task manager 汇报进度
- 处理 retry、skip、pause、resume

### 12.10 `inspection_evaluator`

评测节点和工具。

职责：

- 记录任务状态
- 记录机器人轨迹
- 记录等待时间
- 记录冲突事件
- 输出 CSV / JSON
- 支持批量实验

### 12.11 `inspection_visualization`

可视化包。

职责：

- 发布 RViz marker
- 显示巡检点
- 显示任务状态颜色
- 显示机器人路径
- 显示 reservation table 或局部冲突区域
- 显示统计信息

## 13. 建议 ROS 接口

第一版接口可以简化，后续再正规化。

### 13.1 话题

```text
/inspection/tasks
/inspection/task_status
/inspection/robot_states
/inspection/assignments
/inspection/planned_paths
/inspection/metrics
/inspection/events
/inspection/markers
```

多机器人 Nav2 action 命名空间示例：

```text
/robot_1/navigate_to_pose
/robot_2/navigate_to_pose
/robot_3/navigate_to_pose
```

机器人状态示例：

```text
/robot_1/odom
/robot_1/amcl_pose
/robot_1/scan
/robot_2/odom
/robot_2/amcl_pose
/robot_2/scan
```

### 13.2 `InspectionTask.msg` 草案

```text
string task_id
string task_type
geometry_msgs/PoseStamped target_pose
int32 priority
float64 earliest_start_time
float64 deadline
float64 expected_duration
string status
```

### 13.3 `RobotState.msg` 草案

```text
string robot_id
string status
geometry_msgs/PoseStamped current_pose
string current_task_id
float64 battery_level
bool available
```

### 13.4 `TaskAssignment.msg` 草案

```text
string task_id
string robot_id
int32 sequence_index
float64 planned_start_time
float64 planned_finish_time
```

### 13.5 `TimedWaypoint.msg` 草案

```text
geometry_msgs/PoseStamped pose
float64 arrival_time
float64 leave_time
bool wait_required
```

### 13.6 `MissionMetrics.msg` 草案

```text
float64 makespan
float64 total_path_length
float64 average_waiting_time
int32 completed_tasks
int32 failed_tasks
int32 conflict_count
int32 replanning_count
```

## 14. 数据配置格式

### 14.1 场景 YAML

示例：

```yaml
scenario_name: mvp_2robots_8tasks
map: maps/office_small.yaml
robots:
  - id: robot_1
    start: [0.0, 0.0, 0.0]
  - id: robot_2
    start: [4.0, 0.0, 3.14]
tasks:
  - id: task_001
    type: inspect_point
    target: [1.0, 2.0, 0.0]
    priority: 1
  - id: task_002
    type: inspect_point
    target: [3.0, 2.5, 0.0]
    priority: 1
dynamic_events:
  - time: 60.0
    type: add_task
    task:
      id: urgent_001
      type: inspect_point
      target: [5.0, 1.0, 0.0]
      priority: 5
```

### 14.2 实验配置 YAML

示例：

```yaml
experiment_name: planner_comparison_corridor
scenario: scenarios/corridor_conflict.yaml
robot_counts: [2, 3]
task_counts: [10, 20]
allocation_methods:
  - nearest
  - earliest_finish_time
planning_methods:
  - independent_astar
  - prioritized_planning
  - cbs
repeat: 5
output_dir: experiments/results/
```

## 15. 算法路线

### 15.1 地图建模

第一版使用 2D occupancy grid。

建模步骤：

1. 读取 `.yaml` 和 `.pgm`
2. 将 free cell 作为可通行节点
3. 将 obstacle 和 unknown 作为不可通行
4. 根据机器人半径膨胀障碍物
5. 构建 4 邻接或 8 邻接图
6. 将巡检点投影到最近 free cell
7. 将机器人当前位置投影到最近 free cell

第一版建议用 4 邻接，便于冲突检测。后续再考虑 8 邻接。

### 15.2 单机器人路径规划

第一版实现 A*。

输入：

- start cell
- goal cell
- occupancy grid

输出：

- cell path

启发函数：

- Manhattan distance，适合 4 邻接
- Euclidean distance，适合 8 邻接

需要支持：

- 障碍物膨胀
- 不可达判断
- 路径长度统计
- path smoothing 可以后做

### 15.3 时间扩展 A*

为了处理多机器人冲突，需要把状态从二维扩展为三维：

```text
(x, y) -> (x, y, t)
```

动作：

- 上
- 下
- 左
- 右
- 等待

约束：

- 某时刻某格不能被占用
- 某时刻某条边不能被反向占用
- 超过最大规划时间则失败

### 15.4 Reservation Table

reservation table 记录已经被其他机器人占用的时空资源。

需要记录：

```text
vertex reservation:
  (x, y, t)

edge reservation:
  (x1, y1, x2, y2, t)
```

冲突规则：

- 如果 `(x, y, t)` 已被占用，新机器人不能进入
- 如果机器人 A 在 t 时刻从 u 到 v，机器人 B 不能在同一时刻从 v 到 u
- 窄通道可以额外建成 corridor resource，只允许一个机器人占用

### 15.5 Prioritized Planning

流程：

1. 给机器人排序
2. 先规划优先级高的机器人
3. 将其路径写入 reservation table
4. 后续机器人基于 reservation table 规划
5. 如果失败，尝试改变优先级或触发重规划

优点：

- 实现简单
- 适合第一版
- 能处理基础冲突
- 方便接入 ROS

缺点：

- 不完备
- 受优先级顺序影响大
- 可能找不到其实存在的解

### 15.6 CBS

CBS 可以作为后期进阶算法和对比实验。

基本思想：

- 高层搜索冲突约束树
- 低层为每个机器人单独规划路径
- 发现冲突后添加约束并分支

用途：

- 展示更强的 MAPF 理解
- 和 prioritized planning 做对比
- 分析性能和解质量差异

不建议第一阶段就实现 CBS。先完成 reservation table 和 prioritized planning。

### 15.7 任务分配算法

第一阶段：

- nearest robot
- earliest finish time

nearest robot：

```text
选择当前距离任务目标最近的可用机器人
```

earliest finish time：

```text
估计每台机器人完成该任务的完成时间，选择完成时间最早的机器人
```

中期：

- auction-based allocation
- Hungarian algorithm

后期可以考虑：

- 动态任务插入
- 滚动时域调度
- 带优先级和 deadline 的调度

## 16. 任务状态机设计

任务状态：

```text
PENDING
ASSIGNED
PLANNED
EXECUTING
SUCCEEDED
FAILED
REASSIGNED
CANCELLED
```

状态含义：

- `PENDING`：任务已创建，尚未分配
- `ASSIGNED`：任务已分配给某台机器人
- `PLANNED`：任务路径已规划完成
- `EXECUTING`：机器人正在执行任务
- `SUCCEEDED`：任务完成
- `FAILED`：任务失败
- `REASSIGNED`：任务因为失败或更优调度被重新分配
- `CANCELLED`：任务被取消

典型状态流：

```text
PENDING -> ASSIGNED -> PLANNED -> EXECUTING -> SUCCEEDED
```

失败状态流：

```text
EXECUTING -> FAILED -> PENDING -> ASSIGNED
```

突发任务状态流：

```text
new urgent task -> PENDING -> ASSIGNED -> PLANNED -> EXECUTING
```

## 17. 机器人状态机设计

机器人状态：

```text
IDLE
ASSIGNED
PLANNING
MOVING
WAITING
BLOCKED
FAILED
RECOVERING
```

状态含义：

- `IDLE`：空闲
- `ASSIGNED`：已有任务但未执行
- `PLANNING`：等待路径规划
- `MOVING`：正在移动
- `WAITING`：因冲突或窄通道等待
- `BLOCKED`：Nav2 执行受阻
- `FAILED`：机器人故障或不可用
- `RECOVERING`：恢复处理中

## 18. 执行层设计

执行层不直接控制底盘，而是调用 Nav2。

执行流程：

1. 接收规划器输出的 timed path
2. 选取关键 waypoint
3. 按顺序发送 `NavigateToPose`
4. 执行前检查是否需要等待
5. 到达 waypoint 后更新进度
6. 如果 Nav2 失败，进入 retry
7. retry 超限后汇报任务失败
8. 触发重规划或重分配

需要处理的问题：

- Nav2 action server 未启动
- 目标点被拒绝
- 目标执行失败
- 机器人偏离计划路径
- timed path 与实际执行时间不一致
- 多机器人等待时间累计误差

第一版可以简化：

- 只在高层 waypoint 之间做冲突规划
- Nav2 局部执行偏差通过重新获取机器人状态修正
- 暂不追求严格实时 MAPF 执行

## 19. 评测体系设计

评测是这个项目区别于普通 demo 的关键。

### 19.1 指标

任务级指标：

- completed_tasks
- failed_tasks
- average_task_wait_time
- average_task_completion_time
- urgent_task_response_time

机器人级指标：

- travel_distance
- moving_time
- waiting_time
- idle_time
- blocked_time
- utilization

系统级指标：

- makespan
- total_path_length
- conflict_count
- replanning_count
- success_rate
- average_waiting_time

算法级指标：

- planning_time
- path_cost
- number_of_expanded_nodes
- number_of_constraints
- failure_rate

### 19.2 实验场景

至少准备：

1. open_area
   - 验证基础调度效率
   - 冲突较少

2. corridor
   - 验证共享走廊冲突
   - 对比是否会正面对撞

3. narrow_passage
   - 验证窄通道资源占用
   - 对比等待策略

4. multi_room
   - 验证多区域任务分配
   - 更接近办公楼场景

5. dynamic_task
   - 验证突发任务插入

6. robot_failure
   - 验证机器人失败和任务重分配

### 19.3 对比实验

任务分配对比：

```text
nearest robot
earliest finish time
auction
Hungarian
```

路径规划对比：

```text
independent A*
prioritized planning
CBS
```

实验变量：

```text
robot_count: 2, 3, 5
task_count: 5, 10, 20, 50
map_type: open_area, corridor, narrow_passage, multi_room
dynamic_task_ratio: 0%, 20%, 40%
robot_failure_ratio: 0%, 10%
```

输出：

```text
experiments/results/*.csv
experiments/results/*.json
experiments/plots/makespan.png
experiments/plots/path_length.png
experiments/plots/wait_time.png
experiments/plots/success_rate.png
experiments/plots/planning_time.png
```

## 20. 阶段路线图

### 阶段 0：项目设计，2 周

目标：

- 明确最终形态
- 明确第一版边界
- 明确接口和包结构

任务：

- 完善 `PROJECT_PLAN.md`
- 创建 `docs/system_design.md`
- 创建 `docs/ros_interfaces.md`
- 创建 `docs/algorithms.md`
- 画系统架构图
- 确定项目英文名和包名前缀
- 确定第一版地图和巡检点数量

验收：

- 能用 5 分钟讲清项目做什么
- 能用 5 分钟讲清模块边界
- 能列出第一阶段任务清单

### 阶段 1：离线算法原型，1 到 1.5 个月

目标：

- 不依赖 ROS，先把算法逻辑跑通

任务：

- 实现 occupancy grid loader
- 实现 grid graph
- 实现 A*
- 实现 wait action
- 实现 reservation table
- 实现 prioritized planning
- 实现 nearest allocator
- 实现 earliest finish time allocator
- 实现离线 simulator
- 实现 metrics logger
- 实现路径可视化

验收：

- 2 台机器人、8 个任务可以离线完成
- 3 台机器人、10 个任务可以离线完成
- 输出 CSV
- 输出路径图
- 冲突检测结果正确
- wait 动作可见

### 阶段 2：ROS 2 Jazzy 工程骨架，1 个月

目标：

- 建立可运行的 ROS 2 工作区

任务：

- 创建 ROS 2 workspace
- 创建 `inspection_msgs`
- 创建 `inspection_description`
- 创建 `inspection_simulation`
- 创建 `inspection_bringup`
- 创建基础机器人模型
- 创建双机器人启动
- 接入 Nav2
- 接入 RViz

验收：

- `colcon build` 通过
- 单机器人 Nav2 可运行
- 双机器人 namespace 可运行
- RViz 可显示地图和机器人
- 两台机器人可以分别接收 Nav2 goal

### 阶段 3：基础任务系统，1 到 1.5 个月

目标：

- 建立任务从创建到完成的完整闭环

任务：

- 实现 scenario loader
- 实现 task manager
- 实现 robot state monitor
- 实现 nearest allocator
- 实现 executor
- 接入 Nav2 action
- 记录任务日志

验收：

- 2 台机器人完成 8 个固定巡检点
- 每个任务状态变化清晰
- Nav2 成功和失败都能被记录
- 任务失败后能重试

### 阶段 4：无冲突规划接入，1.5 到 2 个月

目标：

- 让多机器人不是各走各的，而是经过高层冲突规划

任务：

- ROS 中接入 planner
- 将 map 转换为 grid
- 将 robot pose 转为 cell
- 将 task target 转为 cell
- 输出 timed path
- executor 按 timed path 执行
- 实现 vertex conflict 检查
- 实现 edge conflict 检查
- 实现窄通道等待

验收：

- 共享走廊不正面对撞
- 窄通道只允许一台机器人通过
- RViz 可显示规划路径
- 日志可显示等待原因
- metrics 中记录 waiting time

### 阶段 5：C++ 核心迁移，1 到 2 个月

目标：

- 提升系统软件岗位匹配度

优先迁移：

- A*
- reservation table
- prioritized planner
- executor

任务：

- 学习 `rclcpp`
- 写 C++ A* 离线测试
- 写 C++ planner node
- 写 C++ action client
- 写 C++ unit test
- 保留 Python 评测脚本

验收：

- 至少一个核心 ROS 节点为 C++
- 至少一个核心算法模块为 C++
- C++ 模块有测试
- README 能说明 C++ 和 Python 分工

### 阶段 6：进阶算法和动态场景，1.5 个月

目标：

- 加入更有含金量的场景和算法对比

任务：

- 实现 CBS 基础版本
- 加入 dynamic task
- 加入 robot failure
- 加入 replanning
- 加入 auction 或 Hungarian

验收：

- prioritized planning 和 CBS 可对比
- 动态任务可以插入
- 机器人失败后任务可重分配
- 输出实验结果

### 阶段 7：评测、文档和展示，1 个月

目标：

- 形成可以投简历和面试展示的最终项目

任务：

- 完善 README
- 完善架构文档
- 完善算法文档
- 完善实验文档
- 生成图表
- 录制 demo
- 整理简历描述

验收：

- 新人按 README 可以跑 MVP
- GitHub 首页能看懂项目亮点
- 有 demo 视频
- 有实验图
- 有清楚的阶段总结

## 21. MVP 详细定义

MVP 周期建议：2 个月。

MVP 目标：

```text
2 台机器人在一张已知办公楼地图中完成 8 个固定巡检点，
使用 greedy task allocation 和 prioritized planning，
通过 Nav2 执行，并输出基础评测结果。
```

MVP 必须包含：

- 2 台机器人
- 1 张已知地图
- 8 个巡检点
- nearest 或 earliest finish time 分配
- A*
- reservation table
- prioritized planning
- Nav2 执行
- 任务状态机
- CSV 指标统计
- RViz 可视化

MVP 可以不包含：

- CBS
- C++ 迁移
- 突发任务
- 机器人故障
- 多楼层
- Open-RMF
- Web UI

MVP 验收标准：

- 一条 launch 命令启动系统
- 两台机器人都能执行任务
- 没有明显机器人互撞
- 任务完成后输出结果文件
- README 能说明如何运行

## 22. 学习路线

### 22.1 ROS 2 和 Nav2

需要掌握：

- ROS 2 package
- launch
- parameter
- topic
- service
- action
- TF
- namespace
- lifecycle
- Nav2 bringup
- Nav2 action client
- multi-robot Nav2

### 22.2 C++

项目相关 C++ 学习重点：

- class
- struct
- vector
- queue
- priority_queue
- unordered_map
- shared_ptr
- enum class
- lambda
- CMake
- rclcpp Node
- publisher
- subscription
- action client

不需要一开始学完整高级 C++，先围绕项目写起来。

### 22.3 算法

顺序：

1. A*
2. time-expanded A*
3. reservation table
4. prioritized planning
5. CBS
6. Hungarian
7. auction-based allocation

每个算法要能回答：

- 问题怎么建模
- 输入输出是什么
- 为什么适合这个阶段
- 缺点是什么
- 和其他方法如何对比

## 23. 外部资料校准清单

后续查资料时，可以逐项验证：

### 23.1 Open-RMF

关注：

- fleet adapter 思路
- traffic schedule 思路
- 多机器人资源冲突建模
- 任务调度抽象
- 和 Nav2 的关系

不建议第一版直接接入 Open-RMF。先理解设计思想。

### 23.2 Nav2

关注：

- NavigateToPose action
- waypoint follower
- lifecycle manager
- costmap
- controller server
- planner server
- behavior tree navigator
- 多机器人 namespace 配置

### 23.3 Moving AI MAPF Benchmark

关注：

- MAPF 问题定义
- benchmark 地图格式
- 常见指标
- CBS、prioritized planning 等算法表现

### 23.4 招聘要求

重点看岗位是否要求：

- ROS 2
- C++
- Nav2
- Gazebo
- 多机器人调度
- 路径规划
- 仿真测试
- 系统集成
- Linux
- CMake

用招聘要求反向调整项目展示重点。

## 24. 风险和控制

### 24.1 风险：项目过大

控制：

- 先做 MVP
- 暂不做多楼层
- 暂不做真机
- 暂不接 Open-RMF
- 暂不做 Web 平台

### 24.2 风险：卡在多机器人 Nav2

控制：

- 先离线算法
- 再单机器人 Nav2
- 再双机器人 Nav2
- 最后接调度规划

### 24.3 风险：C++ 卡住

控制：

- 第一版 Python 实现
- C++ 从 A* 开始迁移
- 不一开始全 C++
- 保持 Python 版本作为对照

### 24.4 风险：算法太空

控制：

- 每个算法都要接评测
- 每个算法都要有输入输出
- 每个算法都要能在地图上跑
- 不只写公式和 README

### 24.5 风险：只有 demo 没数据

控制：

- 从阶段 1 就输出 CSV
- 每个实验保留 config
- 每个结果可复现
- 最终必须有图表

## 25. 简历展示版本

中文简历描述草案：

```text
基于 ROS 2 Jazzy 设计并实现室内多机器人巡检仿真平台，支持多机器人任务分配、无冲突路径规划、Nav2 导航执行、任务失败重分配和批量评测。系统基于办公楼已知地图构建巡检任务场景，实现 greedy / earliest-finish-time 等任务分配策略，以及 A*、reservation table、prioritized planning、CBS 等路径规划方法，并对 makespan、总路径长度、等待时间、任务成功率和规划耗时进行对比实验。项目采用 Python 进行算法原型和评测脚本开发，使用 C++ 实现核心规划与执行模块，完成 Gazebo 多机器人仿真和 RViz 可视化。
```

英文简历描述草案：

```text
Designed and implemented a ROS 2 Jazzy based multi-robot indoor inspection simulation platform with task allocation, conflict-free path planning, Nav2 execution, failure recovery, and benchmark evaluation. Implemented task allocation strategies and MAPF-inspired planners including A*, reservation table based prioritized planning, and CBS. Integrated multi-robot Nav2 navigation in Gazebo and evaluated the system using metrics such as makespan, total path length, waiting time, success rate, and planning time. Used Python for algorithm prototyping and experiment automation, and C++ for core ROS 2 planning and execution modules.
```

## 26. 项目成功标准

项目成功不是“能启动很多机器人”，而是满足以下标准：

- 场景明确：室内办公楼巡检
- 核心明确：任务分配、无冲突路径规划、评测
- 系统完整：任务输入到执行反馈闭环
- 工程清楚：包结构、接口、状态机、日志完整
- 技术可信：实现 1 到 2 类代表性算法
- 有对比：不同算法、不同场景、不同任务数量
- 有数据：CSV、图表、实验报告
- 有展示：README、架构图、demo 视频
- 有 C++：核心模块不全是 Python
- 可解释：能说清为什么这样设计

## 27. 近期行动清单

接下来建议按这个顺序做：

1. 给项目定英文名
2. 确定包名前缀，例如 `inspection_`
3. 创建 `docs/system_design.md`
4. 创建 `docs/ros_interfaces.md`
5. 创建 `docs/algorithms.md`
6. 创建 `scenarios/mvp_2robots_8tasks.yaml`
7. 先写离线 `inspection_core`
8. 实现 occupancy grid loader
9. 实现 A*
10. 实现 reservation table
11. 实现 prioritized planning
12. 输出第一张离线路径图
13. 再创建 ROS 2 Jazzy workspace
14. 接单机器人 Nav2
15. 接双机器人 Nav2

## 28. 第一周详细任务

第一周不要急着碰 Gazebo。

建议任务：

1. 整理项目名和 README 草稿
2. 建立 `docs/`、`scenarios/`、`experiments/`、`scripts/` 目录
3. 写 `docs/system_design.md`
4. 写第一版场景 YAML
5. 用 Python 写一个简单 grid map
6. 实现 A*
7. 画出单机器人路径
8. 写一篇 `docs/week1_notes.md` 记录设计取舍

第一周验收：

- 有项目结构
- 有系统设计文档
- 有一个离线 A* demo
- 有一张路径图

## 29. 资料修改建议

你后续查资料时，可以在本文档中标记：

```text
[confirmed] 已确认
[todo] 待查证
[changed] 已根据资料修改
[risk] 有风险
```

建议每次查完一个资料来源，就更新一个小节，而不是等全部查完再大改。

优先查证顺序：

1. ROS 2 Jazzy 多机器人 namespace 和 launch 写法
2. Nav2 Jazzy 多机器人配置
3. Gazebo Harmonic 与 ROS 2 Jazzy 的推荐集成方式
4. Open-RMF 的 fleet adapter 和 traffic schedule 思路
5. Moving AI MAPF benchmark 的指标和地图格式
6. 机器人公司招聘要求中的 C++ / ROS / Nav2 能力关键词

## 30. 最终备注

这个项目值得投入 6 到 12 个月，但前提是始终围绕三条主线：

```text
任务分配
无冲突路径规划
仿真评测体系
```

如果某个功能不能服务这三条主线，就先不要做。

第一阶段的目标不是把最终系统全部搭起来，而是做出一个可信、可复现、可扩展的 MVP。后续所有复杂功能都应该建立在这个 MVP 上。
