# PredictiveNav2：面试复盘题库与项目表达边界

> 最后整理：2026-08-28。本文从项目开发过程中的真实疑问提炼而来，用于面试复习，不是让你背标准答案。
>
> **使用方法：** 每道题先用自己的话回答，再打开对应代码、日志或实验材料核对。回答时只说已完成且能复现的内容；写有“待完成”的问题，只能说明设计计划，不能说成既有能力。

## 0. 当前状态：先避免夸大

截至本文更新：

- 已完成并验证：`/scan` 过滤、时间戳 TF 到 `odom`、顺序欧氏聚类、`/dynamic_obstacles/clusters`、RViz cluster Marker；AMCL + DWB 静态导航基线。
- 跟踪已构建并待运行验证：时间戳 `dt` 检查、CV predict、距离 gate 内的贪心一对一关联、Kalman update，以及真实 `tracks_` 的递增 ID、新生和丢失删除；已定义并发布 `/dynamic_obstacles/tracks`。仍没有 tentative/confirmed、Hungarian/Mahalanobis 或正式 benchmark。
- 待完成：tracks 消息发布、轨迹预测消息、原版 MPPI 基线、`DynamicRiskCritic`、benchmark、真机验证。

面试时可以说“我正在按该路线实现”，但不能把待完成部分说成现有成果。

---

## A. 项目定位与总体架构

### Q1：你的项目到底做什么？

**回答核心：** PredictiveNav2 是一个移动机器人动态障碍物预测与风险感知局部导航项目。目标是从 2D LiDAR 中形成障碍物观测，跨帧跟踪并预测短期位置，再让 Nav2 的 MPPI 局部控制器在选择候选运动轨迹时考虑动态风险。

```text
/scan → cluster → track → prediction → DynamicRiskCritic → Nav2 MPPI → /cmd_vel
```

**证据：** [PROJECT_SPEC.md](../PROJECT_SPEC.md)、[PROJECT_PROGRESS.md](../PROJECT_PROGRESS.md)。

**边界：** 目前完整链路尚未全部实现；不要说已经实现了 MPPI 动态避障。

### Q2：它属于路径规划、导航，还是运动控制？

**回答核心：** 主方向是移动机器人导航与决策/局部规划。项目重点是动态障碍物感知、跟踪、预测和 MPPI 局部轨迹风险评价；不是底层电机伺服、PID、动力学控制项目，也不以全局 A* 路径规划为创新点。

### Q3：全局规划、局部规划和运动控制的区别是什么？

**回答核心：** 全局规划在地图上从起点到终点给大方向；局部规划/决策结合传感器和短期目标决定下一小段轨迹；运动控制把速度或轨迹命令跟踪到电机/底盘执行。项目接入的是局部 MPPI 评分层，底层执行由现有底盘/Gazebo 链路负责。

### Q4：为什么不直接做 SLAM 或运动控制？

**回答核心：** 项目使用已有地图和 AMCL/DWB 基线，把范围聚焦在动态障碍物导致的局部风险问题。SLAM、全局规划、底盘控制都是必要基础，但若同时重写会掩盖动态预测与风险评价这一核心问题。

---

## B. LiDAR 感知、坐标系与 ROS 2

### Q5：一帧 `/scan` 怎样变成 cluster？

**回答核心：** 过滤 NaN/Inf 和无效量程；按原始 beam index 用 `angle_min + index × angle_increment` 转成 `lidar_link` 二维点；在 scan 原始时间戳查询 `odom ← lidar_link` TF；再按相邻 beam 与二维距离做顺序欧氏聚类，输出中心、尺寸、点数。

**证据：** `src/predictive_nav_perception/src/scan_info_node.cpp`；[第 01 模块](../cpp_predictive_nav_book/01_LiDAR到动态障碍物观测/README.md)。

### Q6：为什么跟踪坐标系选 `odom`，不是 `map` 或 `lidar_link`？

**回答核心：** `lidar_link` 随机器人移动，跨帧位置变化混入了机器人自身运动；`map` 会受 AMCL 对 `map → odom` 的修正影响，短时间可能不连续。`odom` 对局部连续跟踪更合适，因此用它比较位置、速度和预测。

### Q7：为什么 TF 查询必须使用 scan 的原始时间戳？

**回答核心：** LiDAR 每一帧对应一个采样时刻。若用“最新 TF”，机器人运动时会把较早的激光点投到错误姿态，误差会被误判为障碍物运动。查不到同一时刻 TF 时应丢弃该帧并计数，而不是复用旧变换。

### Q8：为什么 `/scan` 和 cluster topic 使用 SensorDataQoS？

**回答核心：** 高频传感器流更关注最新数据，不值得排队等待过期数据；传感器 QoS 使用较短队列和 best effort 语义，以降低延迟。发布订阅 QoS 不兼容会导致“topic 存在但收不到消息”，因此接口两端必须匹配。

### Q9：Gazebo 真值能不能拿来做跟踪输入？

**回答核心：** 不能。算法输入只能来自 `/scan`、TF 和 odom；Gazebo actor 真值 Marker 仅可用于 RViz 调试或离线评估标签。若用真值给算法输入，无法说明感知、跟踪和预测真的工作。

---

## C. 跟踪中最重要的真实问题

### Q10：为什么两帧直接相减得到的 `naive velocity` 可能是假速度？

**回答核心：** 两帧中被挑到的 cluster 可能不是同一个物理障碍物；聚类也可能分裂/合并、中心抖动。即使 `dt` 正确，错误对象的位置差除以 `dt` 仍会得到假速度。项目曾观察到 4 m/s、13 m/s 的异常示例，这说明问题不在公式而在跨帧对应关系。

**当前处理：** 教学状态仅接受小于 `debug_max_initial_speed_mps` 的速度作为受控演示种子。

**正式方案：** 第 11 步的新 Track 初始速度设为 0，不让 naive velocity 进入真实轨迹；速度由后续多帧 Kalman update 逐渐估计。

### Q11：最近邻关联为什么仍可能错？

**回答核心：** “离预测最近”只是一种启发式，两个障碍物靠近、交叉、遮挡或预测漂移时，最近 cluster 仍可能属于另一个物体。它不能保证同一个物理对象。

**当前处理：** 最近邻 + 欧氏距离 gate；超出 `association_gate_m` 的候选拒绝更新。

**当前基础版：** 对所有 Track 与 cluster 做距离 gate 内的贪心一对一分配；一个 cluster 每帧最多匹配一个 Track，未匹配 Track 进入 missed 状态，未匹配 cluster 创建新 Track。

**可进阶方向：** 用 Mahalanobis 距离把预测协方差纳入 gate；对比匈牙利算法与贪心最近邻；统计 ID switch、误匹配和漏匹配。即使采用这些方法，2D LiDAR 遮挡场景也无法承诺 100% 不换 ID。

### Q12：为什么不匹配时宁可跳过 update？

**回答核心：** 错误测量会把状态和速度拉向另一个障碍物，伤害往往比短时间不更新更大。可靠做法是保留预测、增加 missed 计数，并等待下一帧证据；第 11 步会给出删除阈值，避免轨迹永远存在。

### Q13：Kalman Filter 在本项目中做了什么？

**回答核心：** 状态是 `[px, py, vx, vy]`。预测阶段用 CV 模型按真实 `dt` 推进状态及协方差；关联成功后以 cluster 中心为二维位置测量，用 innovation 和 Kalman gain 融合预测与测量，同时修正位置、速度和不确定性。

**边界：** 同一 predict-associate-update 循环已接入真实 `tracks_`，但动态场景运行验证尚待完成；ID=0 教学状态仍单独保留，不能把它误当作正式输出接口。

### Q14：LiDAR 只测位置，为什么 Kalman update 能改速度？

**回答核心：** 连续位置创新量能说明旧速度是否持续偏大或偏小。预测协方差含有位置与速度之间的关系，因此位置测量通过 Kalman gain 间接修正速度；它不是单帧直接测得速度。

### Q15：为什么要检查 `dt`，不能假设 10 Hz？

**回答核心：** 真正间隔会受仿真、调度、重启和回放影响。状态转移和速度估计都依赖实际 elapsed time；非正、倒退或过大的 `dt` 必须拒绝并诊断，否则预测会严重错误。

### Q15-A：CV 模型里的 `F`、`Q`、`P` 分别是什么？为什么 LiDAR 只量位置仍能改速度？

**回答核心：** 状态取为 `[px, py, vx, vy]`。`F` 是匀速（CV）状态转移矩阵：它把 `vx × dt`、`vy × dt` 加到位置上；`Q` 是过程噪声，表示真实目标可能加减速，不能把“严格匀速”当作事实；`P` 是对整组状态误差及它们相关性的协方差。

预测使用 `x⁻ = F x⁺`、`P⁻ = F P⁺ Fᵀ + Q`。因为 `F` 把位置和速度联系起来，`P` 中会出现位置—速度的交叉协方差；所以后续位置测量产生 innovation 时，Kalman gain 可以据此同时修正 `vx/vy`。这不是说 LiDAR 单帧直接量到了速度，而是多帧位置证据在模型约束下逐步估计出了速度。

**代码证据：** `make_cv_transition_matrix()`、`make_cv_process_noise_matrix()`、`predict_debug_cv_state()`。

### Q15-B：`P`、`Q`、`R` 的参数怎么设？能不能随便调？

**回答核心：** 不能把它们当成“让轨迹看起来顺滑”的任意旋钮。

- 初始 `P` 表示新生 Track 对位置和速度各有多不确定；本项目的初始速度为 0，但故意给较大的速度方差，表达“初值是 0，不等于确信它静止”。
- `Q` 由 `process_acceleration_stddev_mps2` 控制，表示 CV 模型无法解释的加速度。过小会跟不上转弯/变速，过大会让预测过于发散。
- `R` 由 `measurement_position_stddev_m` 控制，表示 cluster 质心位置观测的噪声。过小会过度追随抖动的 cluster，过大又会忽略真实观测。

**诚实边界：** 当前参数是教学版初值，并非已由真机标定得出的最优值。正式实验应固定场景，记录 rosbag；用静止目标的 cluster 质心方差估计 `R`，用目标转弯/变速残差检验 `Q`，再以 ID switch、位置误差、速度稳定性等指标比较参数，而不是只凭 RViz 观感。

### Q15-C：`innovation`、`S` 和 Kalman gain `K` 分别解决什么问题？

**回答核心：** `innovation = z - Hx⁻` 是“本次 LiDAR 观测与预测差了多少”；但差值大不代表一定异常，因为预测和测量本身都有不确定性。`S = H P⁻ Hᵀ + R` 是 innovation covariance：它把状态预测的不确定性投影到测量空间，再加上测量噪声，说明这种差值正常可能有多大。

`K = P⁻HᵀS⁻¹` 决定对 innovation 修正多少：预测不确定、测量可靠时更信测量；预测可靠、测量噪声大时则更保留预测。回答时不要只背公式，要先说明这个“按不确定性分配信任”的含义。

**进一步设计：** 当前关联 gate 是固定欧氏距离 `association_gate_m`；完成正式 Track 后，可把 `S` 用于 Mahalanobis gating，使 gate 随预测不确定性自适应，而不是永远固定 0.40 m。

### Q15-D：为什么代码不用直接求逆，而用 `LDLT` 求解？

**回答核心：** 数学上 `K = P⁻HᵀS⁻¹`，但工程实现不需要真的显式构造 `S⁻¹`。代码将 `S` 做 `LDLT` 分解，再求解线性方程；这通常更稳定、也避免不必要的矩阵求逆。分解失败或结果不为有限数时，本帧跳过 update 并报错，避免把坏数值写回 Track。

**代码证据：** `update_debug_cv_state()` 中的 `Eigen::LDLT<Eigen::Matrix2d>`、`solver.solve(...)` 与有限值检查。

### Q15-E：为什么协方差更新使用 Joseph Form？

**回答核心：** 它不是“在 Kalman update 之后再额外滤一次”，而是 measurement update 中更新 `P` 的数值稳定写法：

`P⁺ = (I-KH)P⁻(I-KH)ᵀ + KRKᵀ`。

第一项表示测量修正后剩余的预测不确定性，第二项表示测量噪声通过 gain 传到状态后的不确定性。它与简写 `(I-KH)P⁻` 在精确数学下等价，但浮点计算中更容易保持 `P` 的对称性与半正定性；对长期反复更新的 Tracker 更稳健。

### Q15-F：一个正式 Track 的生命周期应该是什么？

**回答核心：** 正式流程不应把一帧 cluster 立刻当成可靠目标：未匹配 cluster 先创建 tentative Track（位置来自观测、速度为 0 且速度协方差较大）；连续命中若干帧后才确认；匹配成功则 predict 后 update；暂时没有匹配到时保留预测并增加 `missed_frames`；超过阈值才删除。这样能减少由 cluster 抖动、短暂遮挡或偶发杂点造成的假 Track 和频繁闪烁。

**当前边界：** 第 11 步已实现创建、`missed_frames` 和超过阈值删除；但暂未实现 tentative/confirmed 确认逻辑，单帧未匹配 cluster 仍可直接创建 Track。面试时应明确区分已实现生命周期与后续增强设计。

### Q15-G：多目标关联怎样避免两个 Track 同时抢同一个 cluster？

**回答核心：** 不能让每个 Track 独立贪心地选最近 cluster，否则两个 Track 可能同时选择同一测量，且会产生 identity switch。应先计算所有 Track—cluster 的可行匹配代价（欧氏距离或后续的 Mahalanobis 距离），对超过 gate 的组合置为不可行，再做一对一全局分配；未匹配 Track 和未匹配 cluster 分别走 missed/birth 分支。

**方案取舍：** 贪心最近邻实现简单、计算轻，但在目标接近和交叉时更容易错；Hungarian 算法可得到全局一对一最小总代价分配。后续应在相同 rosbag 上比较二者的 ID switch、误匹配、漏匹配和运行时间，而不是先宣称某种方法“绝对正确”。

### Q15-H：`max_missed_frames` 默认是 5，为什么还要把负参数钳制为 0？

**回答核心：** `declare_parameter<int>("max_missed_frames", 5)` 中的 `5` 只是在用户没有传入参数时的默认值；启动时仍可用 `--ros-args -p max_missed_frames:=-3` 覆盖。`missed_frames` 是 `std::size_t`，表示帧数量，不能为负。若直接把负的有符号整数转为无符号数，会变成极大的正数，导致 Track 几乎永不过期。

**当前处理：** 大于 0 的配置值才转为 `std::size_t`；0 或负数统一按 0 帧处理。这样 0 表示“第一次 miss 后即可删除”，不会出现无意义的负帧数。

### Q15-I：单个 `ObstacleCluster` 为什么还要额外传入 `current_stamp`？

**回答核心：** 时间戳不在单个 `ObstacleCluster` 内，而在外层 `ObstacleClusterArray.header.stamp`。一帧中的所有 cluster 都源自同一次 `/scan`，共享相同采样时间和 `odom` 坐标系，因此不应在每个 cluster 中重复保存相同 header。

`make_new_track(cluster, current_stamp)` 的 `current_stamp` 不是电脑此刻的 wall-clock 时间，而是当前 cluster 帧的测量时间。新 Track 用它初始化 `first_observation_stamp` 和 `last_observation_stamp`；以后匹配成功时更新 `last_observation_stamp`。它为后续按秒判断轨迹陈旧、预测时间对齐和 rosbag 回放提供依据。

### Q15-J：贪心一对一关联怎样从 `Candidate` 记录到最终匹配？

**回答核心：** 嵌套循环先枚举所有 gate 内的候选对，并把当时的 `track_index`、`cluster_index`、`distance_m` 一起存进 `Candidate`；`std::sort` 只按距离重排这些完整对象，不会改变其中的两个下标。随后按从近到远逐项处理：若该 Track 和 cluster 都未被占用，就记录 `matched_cluster_for_track[track_index] = cluster_index`，并将该 cluster 标为已用。

`matched_cluster_for_track` 用 `int` 而不是 `size_t`，因为它需要用 `-1` 表示“当前 Track 未匹配任何 cluster”。这条赋值语句只是记录已经接受的配对，不负责判断物理身份。

**边界：** 先选局部最短 pair 的贪心策略不保证所有 Track 的总匹配代价最小；两个 cluster 靠近、交叉或遮挡时仍可能误认并导致 ID switch。

---

## D. 预测、MPPI 与动态风险（完成后再使用）

### Q16：轨迹预测和 Track 有什么区别？

**回答核心：** Track 表示目标当前的滤波状态与不确定性；预测是把该状态推到多个未来时间点。未来轨迹必须带时间偏移和不确定性，供机器人候选轨迹在“同一时刻”比较风险。

**当前状态：** 这是第 03 模块计划，尚未实现正式 prediction topic。

### Q17：为什么风险评估必须做时间对齐？

**回答核心：** 不能拿机器人现在的位置与障碍物 1 秒后的预测位置直接比较。应比较机器人候选轨迹在 `t=0.6 s` 的位置，和障碍物也在 `t=0.6 s` 的预测位置；否则距离没有物理意义。

**当前状态：** 属于第 04 模块计划，尚未实现。

### Q18：DynamicRiskCritic 和直接发布 `/cmd_vel` 有何区别？

**回答核心：** critic 不直接控制机器人；它只给 MPPI 的每条候选轨迹增加动态风险 cost。MPPI 与其他 critic 综合评分后仍由 Nav2 输出控制命令。这样不会绕开原有静态碰撞检查和控制器约束。

### Q19：DWB 与 MPPI 在本项目中的关系？

**回答核心：** 当前已验证的是 DWB 静态导航基线；后续会先建立原版 MPPI 基线，再接入 DynamicRiskCritic 并做对照。不能把“安装了 MPPI 依赖”说成“已经运行了动态 MPPI”。

---

## E. 验证、真机与工程边界

### Q20：仿真跑通后，为什么还需要真机？

**回答核心：** 真机存在真实 LiDAR 噪声、时间延迟、反光/遮挡、里程计误差、TF 标定、计算资源和安全限制；Gazebo 跑通只能证明系统在受控模型中可运行，不能证明真实部署可靠。

### Q20-A：为什么 RViz 显示 `/map` 的 Topic 是 OK，却仍报 `No map received`？

**回答核心：** `Topic: OK` 只代表 DDS 发现发布者和类型兼容，并不代表当前订阅者已经收到一条可显示的数据。`/map` 是静态地图，map server 用 transient-local durability 保留最后一份地图；若 RViz 在它发布后才加入、但订阅端使用 `Volatile`，便会发现 topic 却收不到历史地图。将 RViz Map 的 durability 改为 `Transient Local` 后，晚加入者也能立刻拿到地图。

**项目证据：** 2026-09-02 的定位排错中，RViz `Map: No map received` 在改为 `Transient Local` 后立即恢复。该问题是 QoS/启动顺序问题，不是地图文件丢失。

### Q20-B：小车中途停止时，怎样区分安全层急停、控制器未启动与 Nav2 任务失败？

**回答核心：** 先查 action 状态，而不是只看最终速度。`/controller_server` 为 `active [3]` 仅说明节点已进入可工作状态；`/navigate_to_pose/_action/status = 6` 表示这一次任务已 ABORTED。项目日志曾出现 `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`，随后 BT navigator 取消 FollowPath，控制器停止机器人，因此 `/cmd_vel_safe` 变为全零是结果而不是第一原因。

**排查顺序：** action status → launch 中 `bt_navigator`/`planner_server` 日志 → `/cmd_vel` 与 `/cmd_vel_safe` → lifecycle 状态。不要把“安全输出为零”直接误判为 LiDAR 安全层必然触发。

### Q20-C：`/odom`、`/amcl_pose` 和 `map → odom` 数值不同，怎样判断是不是定位错误？

**回答核心：** `/odom` 在 `odom` frame，`/amcl_pose` 在 `map` frame，原始数值不能直接相减。应使用 `map → odom` 将前者转换到 map 后再比较；三者数学自洽只说明 TF 链内部一致，不证明 AMCL 选择了真实地点。还要看 scan 墙体是否贴合静态地图，并利用已知仿真出生点或 Gazebo 真值做外部验证。

**工程取舍：** 已知仿真出生点时，项目在 AMCL active 后发布窄协方差的 `/initialpose`，并对未建图的动态 actor 启用 beam skipping；真机必须按真实初始误差与传感器/里程计噪声重设这些参数。

### Q20-D：RViz 不显示 cluster Marker，是否说明 C++ 聚类算法没有运行？

**回答核心：** 不一定。RViz 的显示项只是订阅某个 ROS 话题的客户端；Marker 不显示可能是算法没运行，也可能是 display 被禁用、类型不对、namespace 被关闭或 Topic 写错。项目曾因 RViz 保存配置时将 `MarkerArray` 的 Topic 写成默认 `visualization_marker_array`，而节点实际发布 `/dynamic_obstacles/cluster_markers`，导致算法仍在运行却没有青色 cluster。

**排查顺序：** 先用 `ros2 topic info /dynamic_obstacles/cluster_markers` 看 Publisher count；有 Publisher 时检查 RViz Topic/Enabled/namespace，无 Publisher 才检查 `scan_info_node` 是否启动。这样避免把“可视化配置错误”误判成“感知算法故障”。

### Q21：怎样证明 DynamicRiskCritic 有效？

**回答核心：** 用相同地图、起终点、actor 路线、速度和重复次数，对比原版 MPPI 与动态风险方案；记录到达率、碰撞/近碰、最小距离、耗时、路径长度、急停或不必要等待。不能只展示一段成功视频。

### Q22：项目的局限性应该怎样主动说？

**回答核心：** 首版仅使用 2D LiDAR 与 CV 模型；遮挡、cluster 分裂合并、近距离交叉会导致关联不确定；CV 对急转弯/突然加速不够准确；风险 critic 的有效性需要 benchmark 与真机验证。主动说明边界比假装“完全解决动态避障”更可信。

### Q23：真机安全怎么做？

**回答核心：** 先限速、小场地、可随时断电/急停、独立静态安全层；动态 critic 不是认证安全系统。真机测试从观测与记录开始，再逐步开放运动，不将未经验证的预测直接用于高速控制。

---

## F. 项目归属、AI 与求职表达

### Q24：如果使用 AI 协助写代码，如何诚实地讲这个项目？

**回答核心：** 可以说 AI 用于加快查资料、搭建初版和解释代码，但自己负责运行验证、阅读理解、参数实验、故障定位和设计取舍。不能声称所有代码都独立从零手写；面试中应能解释实际日志、异常速度、TF/QoS 问题和改动理由。

### Q25：你的个人贡献与“教程复现”如何区分？

**回答核心：** 不靠一句“我原创算法”包装。可展示的贡献是：将 LiDAR 感知、跟踪、预测和 Nav2 风险评分组织成清晰模块；对时间戳、frame、QoS、异常数据做保护；用真机/rosbag/对比实验定位问题并形成证据。最终要用代码、提交记录、实验数据和失败分析支撑。

### Q26：用 90 秒怎样介绍项目？

**回答骨架：**

1. 问题：静态导航只看当前障碍物，面对移动目标可能反应太晚。
2. 方法：2D LiDAR cluster → 多目标 CV 跟踪 → 短期预测 → MPPI 动态风险评分。
3. 工程：统一 `odom`、按原始时间戳查 TF、SensorDataQoS、异常 `dt`/过期预测保护。
4. 验证：仿真基线、动态场景、rosbag/指标；完成后补真机验证。
5. 边界：当前完成到哪一步就说到哪一步，后续会量化关联和风险效果。

---

## G. 本次学习材料的筛选原则

已阅读用户提供的《解释C++结构体.pdf》后半部分，其中关于 CV 状态、假速度、数据关联、协方差、innovation、Kalman gain 与 Joseph Form 的可面试内容，已整理到 **Q10–Q15-G**。

筛选时刻意没有把整段“公式推导课”搬进题库：面试重点应是你能否结合本项目说清楚它解决什么工程问题、代码在哪里实现、当前有什么边界、下一步如何验证。若面试官继续深挖，再展开公式和推导。

## H. 已记录的后续跟踪优化计划（尚未开始实现）

当前学习目录只有第 01–14 步；“第 15 步”不是现有章节，而是根据当前发现的 identity switch 风险新增的待办。不要把它说成已经实现。

建议顺序：

1. 先完成第 12–14 步：发布 Track、RViz 显示 ID/速度、录制并回放 rosbag；
2. 用固定 rosbag 记录贪心 baseline 的 ID switch、误匹配、漏匹配和运行时间；
3. 再新增 `15_关联鲁棒性升级与对比`：Mahalanobis gate、Hungarian 一对一分配、tentative/confirmed Track，以及同输入下的升级前后对比；
4. 先完成第 03、04 模块的端到端仿真主链路，再在真机前依据 rosbag 证据决定升级范围；真机阶段重新标定 `Q/R/gate/max_missed_frames`，不把真机当作第一次排查关联问题的场所。

**面试表达：** “我先用可解释的贪心 baseline 建立端到端链路和可复现 rosbag，再根据近距离交叉场景中的 ID switch 证据升级关联；不一开始堆复杂算法，也不把仿真参数直接当作真机参数。”
