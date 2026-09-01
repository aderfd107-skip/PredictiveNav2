# PredictiveNav2 — 项目总体规格

> **唯一总体设计依据。** 本文档基于工作区实际状态维护，最近一次状态同步为 **2026-08-28**。文中明确标记的 `Not Started` 项目不是现有功能；后续核心实现、重构和 README 均以本文为准。实现与设计不一致时，必须先更新本文并记录原因。

## 1. 项目定位与边界

**最终名称：PredictiveNav2**

**完整定位：** 基于 ROS 2 / Nav2 的动态障碍物预测与风险感知导航系统（Dynamic Obstacle Prediction and Risk-Aware Navigation System）。

名称同时作为 GitHub 展示名称；ROS 2 package、C++ namespace、launch/config 前缀统一使用合法的小写形式 `predictive_nav_*` / `predictive_nav`。旧多机器人巡检语义不再进入当前源码或运行接口。

### 1.1 问题定义

在平面室内环境中，机器人从 2D LiDAR 的 `/scan` 检测局部动态障碍物，维护稳定目标轨迹，并预测短期未来位置与不确定性。该预测结果直接进入 Nav2 MPPI 的候选轨迹评价，使机器人相较于仅依赖静态/当前 costmap 的局部规划能更早减速、等待或选择低风险路径。

核心链路严格限定为：

```text
2D LiDAR → 预处理/聚类 → 多目标跟踪 → CV 状态估计 → 轨迹预测
         → 动态风险与 TTC → 自定义 Nav2 MPPI Critic → 动态环境导航 → Benchmark → 真机迁移
```

不在范围内：多机器人任务分配、MAPF/CBS、配送/巡检业务流程、机械臂、视觉语言导航、端到端学习、强化学习、3D SLAM，或任何将项目变成产品系统的功能。

## 2. 当前代码库审查结果

### 2.1 实际目录与构建状态

当前工作区有六个 `ament_cmake` 包：`predictive_nav_description`、`predictive_nav_simulation`、`predictive_nav_bringup`、`predictive_nav_msgs`、`predictive_nav_perception` 与 `predictive_nav_tracking`。`predictive_nav_msgs` 定义了 `ObstacleCluster`、`ObstacleClusterArray`、`TrackedObstacle` 与 `TrackedObstacleArray`。感知节点 `scan_info_node` 可用 `SensorDataQoS` 订阅 `/scan`，按 ROS 参数筛除非有限和量程外读数，将有效距离转为 `lidar_link` 二维点，并以原始 scan 时间戳通过 TF 转为 `odom` 跟踪点；它还实现了顺序欧氏聚类，以 `SensorDataQoS` 向 `/dynamic_obstacles/clusters` 发布每帧几何观测（中心、轴对齐尺寸、点数、公共 `odom` header），并在独立的 `/dynamic_obstacles/cluster_markers` 发布只供 RViz 调试的 MarkerArray。它尚未判断 cluster 是否动态。`predictive_nav_tracking` 已以兼容 QoS 订阅该 cluster topic、验证连续 header 的 `dt`，并实现基础版真实多目标 Track 循环：CV 状态/协方差预测、距离 gate 内的贪心一对一关联、二维 Kalman update、递增 ID、新生 Track、`missed_frames` 与过期删除；新 Track 从零速度和较大速度协方差开始。它现以 `SensorDataQoS` 发布 `/dynamic_obstacles/tracks`，将二维位置/速度及其协方差、尺寸和生命周期字段交给下游；教学用单目标状态仍保留用于观察差分速度和滤波过程。未实现 tentative/confirmed、动态性分类、Hungarian/Mahalanobis 或运行期 benchmark，因此不能宣称已可靠解决遮挡/交叉下的 ID switch。`predictive_nav_bringup` 已提供 AMCL + DWB 已知地图静态导航 baseline。`predictive_nav_simulation` 已包含三个可控 Gazebo 动态 actor；它们通过 ROS–Gazebo `set_pose` 服务运动，并被 `/scan` 实际观测到。轨迹预测、动态风险和 Nav2 plugin 仍未实现。

初始审查（2026-08-17）曾执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_description predictive_nav_simulation
```

当时两个基础包均构建成功。此后已加入 `predictive_nav_bringup` 和动态 actor 控制节点；2026-08-20 重新构建 `predictive_nav_simulation` 与 `predictive_nav_bringup` 已通过。环境是 ROS 2 **Jazzy**，而非旧文档中提及的 Humble。当前系统已安装 `nav2_mppi_controller`；本机头文件确认它可通过 `mppi::critics::CriticFunction::initialize()` 和 `score(mppi::CriticData&)` 扩展 Critic，因此首选该路径。

### 2.2 当前已实现功能（非规划）

| 分类 | 实际内容 | 状态 |
|---|---|---|
| 机器人模型 | 差速底盘、轮子、2D LiDAR、IMU、可选摄像头外观、可选货箱/传感器杆 | Implemented |
| Gazebo 仿真 | `dynamic_navigation_lab.sdf`、单自车 spawn、Gazebo Harmonic 插件、3 个可复位动态 actor | Implemented |
| ROS 接口 | 根话题 `/scan`、`/odom`、`/imu`、`/joint_states`、`/tf`、`/cmd_vel`，以及 `/cmd_vel_safe` 执行链路 | Implemented |
| 可视化 | Nav2 RViz、SDF 静态模型 Marker、动态 actor 的真值调试 Marker | Implemented |
| 基础安全过滤 | Python `scan_safety_guard.py` 按 LaserScan 最小距离限速 | Implemented，但不是预测风险算法 |
| SLAM | 通用 `mapping.launch.py`、已保存 PGM/YAML 地图 | Implemented，仅作地图制作/调试 |
| 动态障碍物 | 3 个固定路线 actor；LiDAR cluster 与基础 CV 多目标 Track 生命周期已构建；无未来轨迹预测或风险控制 | Actor/聚类/Track baseline Implemented；运行期评估待完成 |
| Nav2 导航 | AMCL + DWB 已知地图 baseline、初始位姿确认、Nav2 Goal RViz 工具与受控启动等待；直接启动 Nav2 节点，不依赖缺失的 `nav2_bringup` | Implemented（已完成单目标端到端仿真验证） |
| Benchmark / 测试 | 无自动实验、CSV/JSON 记录、单元测试或集成测试 | Not Started |

### 2.3 文件分类与处理结论

分类只表示第一轮重构的目标；`REMOVE` 内容必须在查清引用并同步更新 CMake、package.xml、launch、配置和文档后才删除，绝不迁入 `legacy/`。

| 分类 | 路径/内容 | 审查结论与处理 |
|---|---|---|
| KEEP | `predictive_nav_description/urdf/{base,wheels,inertials,sensors,gazebo_plugins}.xacro` | 差速模型、LiDAR、IMU、Gazebo odom/scan 链路已重命名并保留。 |
| IMPLEMENTED | `predictive_nav_description`、单一 `display.launch.py`、RViz | 已统一为中性单机器人显示与标准 frame/topic。 |
| REMOVED | 可选载荷、角色外观、重复显示 launch 与角色参数 | 已删除；它们不属于动态预测导航。 |
| KEEP | `dynamic_navigation_lab.sdf` 的墙体、走廊、窄通道、基础静态家具布局 | 已保留并改为中性动态导航实验场景。 |
| IMPLEMENTED | `predictive_nav_simulation`、实验 launch、地图/RViz/marker 脚本 | 已收敛为单自车与标准根话题接口；三个动态 actor 默认关闭，仅在动态试验显式启用。 |
| IMPLEMENTED | `mapping.launch.py`、`mapping.yaml`、现有 PGM/YAML | 已改为通用地图制作工具，使用 `odom`、`base_footprint` 与 `/scan`。 |
| REFACTOR | `scan_safety_guard.py` | 保留为可关闭的手动仿真急停层；MPPI benchmark 不应依赖它。 |
| REMOVED | 旧业务规划文档与无引用 TF 快照 | 已删除。 |
| KEEP | `docs/debug_log.md`、`docs/run_checklist.md`、`docs/office_service_map_sketch.md` | 原有文档内容按用户要求完整保留；它们是历史调试/设计资料，不纳入本轮删除或覆写范围。新增的动态导航场景说明仅作补充。 |

`build/`、`install/`、`log/` 是构建产物；不作为源码架构的一部分。应确认 `.gitignore` 覆盖它们，且不能将它们用作实现状态证据。

### 2.4 已发现的工程风险

- 已知地图导航时，AMCL 是唯一的 `map → odom` 发布者；不得同时启用 `world → odom` static anchor，否则 `odom` 会拥有两个父 frame，造成激光、定位和控制错位。mapping launch 与 nav baseline 已分别显式处理该开关。
- 当前 `/odom` 来自 Gazebo 真值 OdometryPublisher；它适合稳定的仿真基线，但不应被误称为真实机器人里程计。wheel odom 仍仅为调试话题。
- 动态 actor 通过 Gazebo `set_pose` 作运动学控制，且其 RViz 真值 Marker 只用于调试。感知、跟踪、预测和 Critic 严禁订阅该真值话题，必须只从 `/scan` 和 TF/odom 推导障碍物状态。
- `scan_safety_guard.py` 的 LaserScan 回调与 timer 共享距离/速度状态，没有明确数据一致性设计；在单线程 executor 中通常可用，但不可作为实时导航关键安全架构。
- launch、URDF、脚本中有重复的机器人描述构造逻辑和旧命名，重命名需要一次性处理路径、包依赖、`FindPackageShare`、xacro `find`、安装规则和 docs。

## 3. 目标架构与 ROS 数据流

### 3.1 推荐 package 划分

第一轮按算法边界创建/重命名如下包；避免将每一个小类拆成过多包。

```text
src/
  predictive_nav_description/       # URDF/Xacro 与 RViz 机器人显示
  predictive_nav_simulation/        # Gazebo world、动态 actor、桥接、地图制作
  predictive_nav_bringup/           # Nav2、定位、感知、tracking、controller 启动编排
  predictive_nav_msgs/              # Cluster、TrackedObstacle、PredictedTrajectory 接口
  predictive_nav_perception/        # C++：scan 预处理、TF、Euclidean clustering
  predictive_nav_tracking/          # C++：关联、CV Kalman filter、track 生命周期
  predictive_nav_prediction/        # C++：短期轨迹和协方差传播
  predictive_nav_mppi/              # C++：DynamicRiskCritic plugin 与风险工具
  predictive_nav_benchmark/         # Python：重复实验、rosbag、指标导出、绘图
```

`predictive_nav_localization` 暂不独立建包：Jazzy 的 `slam_toolbox`、AMCL 与 `robot_localization` 由 `predictive_nav_bringup` 参数化调用，除非随后出现自有融合代码。

### 3.2 数据流

```text
/scan + TF(base_link←scan) ──> perception ──> /dynamic_obstacles/clusters
                                                   │
/odom, /tf ───────────────────────────────────────┘
                                                   ↓
                                            tracking (CV KF)
                                                   ↓
                                  /dynamic_obstacles/tracks
                                                   ↓
                                            prediction
                                                   ↓
                        /dynamic_obstacles/predictions ──> DynamicRiskCritic
map/AMCL/SLAM ──> Nav2 global planner ──> MPPI candidate trajectories ──> /cmd_vel
                                                                      ↓
                                                       benchmark recorder / rosbag
```

所有内部障碍物状态以 `odom` 为在线局部控制 frame，遵循时间戳；全局 benchmark 可通过 `map → odom` 转换记录。感知节点需要在 scan 的原始 stamp 查询 `odom ← scan`，不可使用“最新 TF”。无法变换、过期、乱序或非有限 scan 数据要丢弃并诊断计数，不能生成错误障碍物。

## 4. 模块、接口与 QoS

| 模块 | 订阅 | 发布 | 核心职责 |
|---|---|---|---|
| Perception | `/scan` (`SensorDataQoS`)、`/tf`/`/tf_static` | `/dynamic_obstacles/clusters` | 距离/角度裁剪、去除无效点、坐标变换、Euclidean 聚类、cluster 特征。 |
| Tracking | `clusters` | `/dynamic_obstacles/tracks` | gating + 最近邻、KF predict/update、ID、miss 生命周期。 |
| Prediction | `tracks` | `/dynamic_obstacles/predictions` | 将各 active track 的位置及 2×2 位置协方差离散预测至 horizon。 |
| MPPI Critic | `predictions`（生命周期节点内可靠最新缓存） | 无独立控制 topic | 评分每条 MPPI 候选，向 `CriticData.costs` 累加动态风险/TTC。 |
| Nav2 | map/TF/odom/scan/costmap | `/cmd_vel` | 全局路径、原版静态避障、MPPI 控制。 |
| Benchmark | `/odom`、tracks、predictions、`/cmd_vel`、Nav2 action/diagnostics、事件 | CSV/JSON、rosbag | 采样指标、场景元数据和实验结果。 |

建议的 `predictive_nav_msgs`：

- `Cluster.msg`：`std_msgs/Header header`、`uint32 point_count`、`geometry_msgs/Point centroid`、`geometry_msgs/Vector3 size`、`geometry_msgs/Point[] points`（可配置是否发布点，默认关闭以减负）。
- `TrackedObstacle.msg`：`uint32 track_id`、`PoseWithCovariance pose`、`TwistWithCovariance twist`、`Vector3 size`、`uint32 age`、`uint32 missed_frames`、`float32 confidence`；时间与 frame 在外层数组 header 中共享。
- `PredictedTrajectory.msg`：`Header`、`uint32 track_id`、`builtin_interfaces/Duration[] offsets`、`PoseWithCovariance[] poses`、`Vector3 size`、`float32 confidence`。
- `ClusterArray`、`TrackedObstacleArray`、`PredictedTrajectoryArray` 包装数组并带共同 `Header`。

动态数组用 `SensorDataQoS`（best effort、短队列）从感知到预测，MPPI 缓存采用最新有效、时间阈值（`max_prediction_age`）策略。若预测过期，Critic 不得静默沿用；应计数、节流告警，并根据参数选择“跳过动态评分”或“保守停止”。

## 5. 感知、跟踪与预测算法

### 5.1 2D LiDAR 动态障碍物检测

首版采用可解释的 Euclidean 聚类：

1. 验证 `angle_increment`、ranges、时间戳和 TF；过滤 NaN/Inf、超量程、机器人自体半径内点和可选地面/静态 mask 区域。
2. 将 scan 点转换到 `odom`，按相邻/半径阈值聚类（阈值可随量程线性放宽）。
3. 删除点数过少、尺寸过大/过小或在已知静态 costmap 高占据区域的 cluster；后者仅为可配置辅助，不把地图误差当作动态目标。
4. 输出 centroid、轴对齐宽高、点数、原始/估计观测协方差。

首版不引入 DBSCAN。只有在可重复实验表明变密度 scan 使 Euclidean 聚类显著失败时，才增加 DBSCAN 作为可对比实现。

### 5.2 多目标跟踪与数据关联

每帧先对所有 track 用实际 `dt` 预测，再构造 cluster-track 欧氏距离或 Mahalanobis 距离矩阵。超过 `association_gate_m` 或卡方门限的配对设为不可用；在保留配对中采用确定性的最近邻贪心匹配。匹配目标 update，未匹配观测创建新 ID，未匹配 track 增加 missed count；超过 `max_missed_frames` 或 `max_track_age_without_update_s` 删除。重现的短暂目标可在删除阈值前保持 ID，超过阈值则视为新目标。

需要处理 `dt <= 0`、超过 `max_dt_s`、速度/协方差非有限、时间倒退和观测异常；此时拒绝本帧或裁剪 `dt` 并记录诊断。首版完成并测试后，才可增加匈牙利算法与 greedy 的 benchmark 对比。

### 5.3 Kalman filter（Constant Velocity）

状态、观测为：

\[
x=[p_x,p_y,v_x,v_y]^T,\quad z=[p_x,p_y]^T
\]

\[
F(dt)=\begin{bmatrix}1&0&dt&0\\0&1&0&dt\\0&0&1&0\\0&0&0&1\end{bmatrix},\quad
H=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}
\]

对白噪声加速度方差 \(q=\sigma_a^2\)：

\[
Q=q\begin{bmatrix}
dt^4/4&0&dt^3/2&0\\0&dt^4/4&0&dt^3/2\\dt^3/2&0&dt^2&0\\0&dt^3/2&0&dt^2
\end{bmatrix}
\]

测量噪声 \(R=diag(\sigma_{mx}^2,\sigma_{my}^2)\)。代码用 Eigen 固定尺寸矩阵实现：

\[
x^- = Fx,\ P^- = FPF^T+Q;\quad S=HP^-H^T+R;\quad K=P^-H^TS^{-1}
\]

\[
x=x^-+K(z-Hx^-),\quad P=(I-KH)P^-(I-KH)^T+KRK^T
\]

最后采用 Joseph form 保持半正定性；每次更新检查 `S` 可逆、矩阵有限性和对称性。初始化 `P` 明确给位置/速度较大不确定性，速度限幅只用于拒绝明显异常观测，不可替代协方差建模。

### 5.4 短期轨迹预测

参数 `prediction_horizon_s`、`prediction_dt_s`、`max_prediction_age_s`、`max_track_covariance` 全部 ROS 参数化。对每个 active track，在 \(t_k=k\Delta t\) 输出：

\[
\hat{x}_k=F(t_k)\hat{x}_0,\qquad P_k=F(t_k)P_0F(t_k)^T+Q(t_k)
\]

发布二维位置与其 2×2 协方差；超出协方差阈值或低 confidence 的轨迹不供风险 critic 使用。预测绝非仅 RViz Marker，而是 Critic 的唯一动态障碍物输入。

## 6. Nav2 / MPPI 风险集成

### 6.1 选择与依赖

保留 Nav2 的 global planner、costmap、BT、AMCL/SLAM 等成熟模块。baseline 先分别稳定运行 DWB 和原版 MPPI。当前 Jazzy 的公开头文件表明 `mppi::critics::CriticFunction` 为可行扩展点，因此实现 `predictive_nav_mppi::DynamicRiskCritic`，由 pluginlib 加载到 MPPI `critics` 列表中；仅当该接口在实际构建/运行中不能满足带时序预测的输入时，才评估独立 `nav2_core::Controller`，并在本节记录理由。

Critic 在 lifecycle `initialize()` 获取参数、节点 logger 和动态预测订阅；`score(CriticData&)` 对 `data.trajectories` 的每一采样轨迹、每一时间点计算成本并累加到 `data.costs`。它不得发布 `/cmd_vel`、阻塞等待消息、执行 TF 查询或在控制周期中分配大量内存。预测缓存以 mutex/原子快照实现，生命周期 deactivate/cleanup 时安全清空。

### 6.2 风险函数

设 MPPI 候选轨迹第 \(i\) 条在时刻 \(t_k\) 的机器人中心是 \(p_{r,i,k}\)，障碍物 \(j\) 预测均值和位置协方差为 \(\mu_{j,k},\Sigma_{j,k}\)。

不确定性安全半径：

\[
r_{safe,j,k}=r_r+r_{o,j}+d_0+k_\sigma\sqrt{\lambda_{max}(\Sigma_{j,k})}
\]

其中 \(r_r\) 为机器人 footprint 等效半径，\(r_{o,j}\) 由 cluster 尺寸给出，\(d_0\) 为基础动态余量。定义 \(d=\Vert p_r-\mu\Vert\)，平滑距离项：

\[
\phi(d,r)=\begin{cases}
C_{collision},&d\le r\\
\left(\frac{r_{influence}-d}{r_{influence}-r}\right)^2,&r<d<r_{influence}\\
0,&d\ge r_{influence}
\end{cases}
\]

\[
J_{dynamic,i}=w_{dynamic}\sum_k\gamma^k\sum_j c_j\,\phi(d_{i,j,k},r_{safe,j,k})
\]

其中 \(c_j\) 是 track confidence，\(\gamma\in(0,1]\) 是时间折扣。若预测点早于启动时间、超过 max age 或协方差失效，则不使用。

相对位置 \(r=p_o-p_r\)、相对速度 \(v=v_o-v_r\)，在 \(r\cdot v<0\) 时：

\[
TTC=-\frac{r\cdot v}{\Vert v\Vert^2+\epsilon}
\]

仅在 \(0<TTC<T_{h}\) 且预测的最近距离低于 \(r_{safe}\) 时加成本：

\[
J_{ttc,i}=w_{ttc}\sum_{j}\max(0,1-TTC_j/T_h)^2
\]

实际实现优先以离散候选轨迹的首次间距穿越时间为准，解析 TTC 用于低开销预筛选。总 MPPI 目标为：

\[
J=w_{path}J_{path}+w_{goal}J_{goal}+w_{static}J_{static}+J_{dynamic}+J_{ttc}
\]

`w_dynamic`、`w_ttc`、`d0`、`k_sigma`、`r_influence`、`ttc_horizon_s`、`collision_cost`、时间折扣、最大参与 track 数量均以参数暴露。采用有限值检查、\(\epsilon\)、成本上限和可重复的固定排序，防止 NaN、爆炸成本或非确定性。

### 6.3 预期行为

系统应在以下场景中体现预测优势：横穿路径时提前降速；迎面运动时扩大风险区并选更安全路径；窄通道会车时等待；暂时阻塞时停止而非贴近振荡；多个交叉动态目标时保持足够间距。不能以“障碍物刚写入 costmap 后绕开”作为项目成功标准。

## 7. 仿真与 Benchmark

### 7.1 场景

当前办公室几何已保留，并已实现三个基础 actor 路线：下方通道竖直横穿、中央主通道水平横穿、右上房间水平横穿。它们用于开发期的传感器、检测和跟踪验证；静态 baseline 默认关闭 actor，动态试验显式启用。下一阶段要将 actor 数量、初始状态、路线、速度、机器人起终点与持续时间 YAML 化，形成以下最低 benchmark 场景集：

1. 单动态障碍物横穿全局路径；
2. 自车与动态障碍物迎面；
3. 窄通道动态会车；
4. 动态障碍物静止阻塞后离开；
5. 多个障碍物交叉运动；
6. 静态空场景（回归和控制耗时基线）。

每个场景用 seed、初始 pose、目标 pose、actor 半径/轨迹/速度和时长 YAML 化，确保重复可复现。

### 7.2 对照与消融

必须比较：DWB、Nav2 原版 MPPI、启用 `DynamicRiskCritic` 的 PredictiveNav2 MPPI。固定机器人 footprint、地图、目标、障碍物 seed、最大速度和终止条件。

消融顺序：原版 MPPI → 仅当前障碍物位置风险 → 加 CV 速度预测 → 加 TTC → 加预测协方差。每个条件多 seed 重复，报告均值、标准差、样本数和失败原因，而非单段演示视频。

### 7.3 指标与导出

每次运行记录 CSV/JSON：导航成功率、碰撞次数/率、最小机器人-障碍物距离、完成时间、路径长度、线/角速度平滑度（加速度或 jerk RMS）、停止次数、恢复次数、MPPI/控制周期耗时、CPU 使用率、预测 age 和有效 track 数。rosbag2 同步保存 `/scan`、TF/odom、tracks、predictions、`/cmd_vel`、Nav2 diagnostics/action 状态；Python 汇总脚本输出带配置和 git revision 的结果目录及图表。

## 8. 真机迁移接口

当前阶段必须不依赖真实硬件运行。真机阶段仅替换 `predictive_nav_simulation` 的传感器来源，维持：

```text
/cmd_vel                 geometry_msgs/Twist（Nav2 输出）
/odom                    nav_msgs/Odometry
/imu                     sensor_msgs/Imu
/scan                    sensor_msgs/LaserScan
/tf, /tf_static          map → odom → base_link → laser/imu
robot_localization       wheel odom + IMU → 滤波 odom / TF
```

硬件适配必须明确 frame_id、时间同步、QoS、base footprint、轮径/轮距、IMU 协方差和紧急停止责任。仿真的 Gazebo 真值 odom 不得成为跟踪或 Critic 的隐式依赖。

## 9. 测试、质量与状态管理

### 9.1 最低测试集

- C++/gtest：KF predict/update 与变量 dt、Joseph covariance、门控/最近邻关联、ID 生命周期、预测协方差增长、TTC 边界、动态风险单调性/有限性。
- ROS 集成测试：合成 LaserScan → cluster → tracks → predictions；固定 TF 与时间戳；MPPI Critic plugin 加载和单控制周期不崩溃。
- 仿真回归：每个 benchmark scenario 至少一条 deterministic smoke test；全量实验由 benchmark runner 执行。
- 静态检查：`ament_lint_auto`、编译警告、参数范围验证、生命周期与线程安全检查。

核心实时 C++ 使用 C++17、Eigen、RAII、智能指针、const correctness；Python 仅用于 actor/实验/统计等非实时用途。所有日志采用节流，所有参数 validate 后使用。

### 9.2 实现状态与更新规则

| 模块 | 状态 |
|---|---|
| 仓库审查与本规格 | Implemented |
| 命名/第一轮结构重构 | Tested |
| 单机器人动态仿真场景 | Implemented（3 个 actor 已完成最小运行与 `/scan` 观测验证；尚未 YAML 化和自动化） |
| LiDAR 聚类 | Implemented（C++ 感知节点已发布 `odom` cluster；动态性判断与系统化测试待完成） |
| 多目标跟踪 + CV KF | Implemented baseline（真实 ID、CV predict/update、贪心一对一关联、birth/miss/expiry、tracks topic 已构建；运行期评估与进阶关联待完成） |
| 轨迹预测 | Not Started |
| Nav2 DWB/原版 MPPI baseline | Implemented（DWB 已完成最小端到端验证；原版 MPPI 尚未开始） |
| DynamicRiskCritic | Not Started |
| Benchmark 与结果导出 | Not Started |
| 真机适配 | Not Started |

每个模块仅可按 `Not Started → In Progress → Implemented → Tested` 更新；`Implemented` 表示代码和最小运行验证存在，`Tested` 还要求对应可重复测试通过。设计变更需追加“变更原因、影响模块、验证结果”。

## 10. 第一轮重构执行顺序

1. **完成：** 已将基础包重命名为 `predictive_nav_description`、`predictive_nav_simulation`，并更新 package/launch/xacro/install 引用。
2. **完成：** 已合并显示 launch，去除业务角色、双机器人默认 spawn 与未使用外观代码。
3. **完成：** 已将世界、地图、RViz、SLAM 工具和运行文档改成中性动态导航实验命名，并保留单自车 LiDAR/IMU/TF/odom 链路。
4. **完成：** 已删除已确认无关文档和无引用生成 TF 快照；下一步重新构建并验证重命名包。
5. 先建立 `predictive_nav_msgs` 与 perception 的独立测试，再依赖顺序实现 tracking、prediction、原版 MPPI baseline、Critic、benchmark。

在第 1–4 步完成并构建验证前，不创建平行旧/新包，不实现新的核心算法，也不删除任何尚未验证引用关系的内容。

### 10.1 实施记录

**2026-08-17 — 第一轮结构重构完成。** 原因是已有实现仍以多机器人业务语义组织，无法作为动态预测导航的清晰基线。完成包、launch、URDF、地图、RViz 和 SLAM 工具的统一重命名；移除业务角色、冗余显示入口、旧规划文档和无引用 TF 快照。`docs/` 原有内容随后按用户要求恢复并保留，后续仅能增量维护。验证结果：两个基础包 `colcon build` 通过，xacro/URDF 与 SDF XML 解析通过，三个 launch 均可加载并列出参数。动态 actor 及所有核心算法保持 `Not Started`，未被本轮结构变更伪装为已实现。

**2026-08-19 — 远端历史整合。** 将远端新增提交 rebase 到当前分支后，确认其 `inspection_core` 包仅包含多机器人 A*/任务规划代码，且无其他包引用；已从源码树移除以保持项目边界。根据用户要求，`docs/inspection_core_explained.md` 及其他既有文档保留为历史资料，不删除。

**2026-08-19 — DWB 静态导航 baseline 实施与最小验证完成。** 基于已有 `scan`、`odom`、TF 与静态地图，新建 `predictive_nav_bringup`，加入 `nav_baseline.launch.py`、AMCL + DWB 参数、导航 RViz 和等待 map/scan 后确认 AMCL 收敛的初始位姿节点。运行时直接启动 Nav2 lifecycle nodes，不依赖未随当前 Jazzy `navigation2` 元包安装的 `nav2_bringup`。验证：三个包构建及 launch 参数加载通过；两套 lifecycle manager 均报告受管节点 Active；在 Gazebo 中向 `NavigateToPose` 发送 `(5.8, -2.5)`，机器人从约 `(5.76, -3.80)` 行驶至约 `(5.81, -2.69)`，动作返回 `SUCCEEDED`、零 recovery。该状态仅表示 DWB 的最小端到端运行已存在；三目标回归、原版 MPPI、动态算法和 benchmark 仍未实现。

**2026-08-20 — 动态 actor 场景实现与规格同步。** 在现有静态地图之外加入三个可控橙色方块，覆盖下方、中央和右上通行区域；每个 actor 可通过 ROS 2 服务 start/stop/reset，并由 Gazebo `set_pose` 运动学控制。验证：三个 actor 的模型与控制节点均构建通过；单 actor 运行时确认 set_pose 服务连接成功、位姿随时间改变，且 `/scan` 中的红色激光点随其移动。RViz 的半透明真值框只用于核对场景，严格不作为算法输入。本次同步将过时的双机器人接口、无 actor 表述和 TF 风险替换为当前实际状态；LiDAR 聚类、跟踪、预测、原版 MPPI、DynamicRiskCritic 与 benchmark 仍为 `Not Started`。

## 11. 最终 README 应包含

README 必须展示：项目问题与边界、系统/数据流图、CV-KF 与动态风险/TTC 公式摘要、包结构、仿真启动方法、三套 baseline 与消融命令、场景与指标定义、结果表/图、rosbag 可复现实验、已知限制、真机接口和引用版本。不得把计划功能展示为已实现，也不得以旧多机器人巡检叙事替代动态预测导航贡献。
