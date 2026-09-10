# 10：RViz 显示预测轨迹

这一小步不改变预测算法，也**不控制 Nav2**。它把第 09 步已经发布的预测消息，转换成便于人观察的 RViz 图形。

完成后，画面中应出现：

- **青色折线**：同一个 Track 从当前时刻向未来 `2.0 s` 的 CV 预测位置；
- **青色圆点**：每隔 `0.2 s` 的一个预测采样点；
- **紫色细椭圆轮廓**：最后一个预测点处的旋转 2σ 位置不确定性范围。

这比给每个目标不断打印 `ID / 速度 / miss` 更适合看整体运动趋势。文字信息仍可通过 Track Marker 或终端日志查看。

## 先区分三种东西

```text
/dynamic_obstacles/tracks
        ↓
prediction_node
        ├── /dynamic_obstacles/predictions        正式算法输出
        └── /dynamic_obstacles/prediction_markers 只给 RViz 看的图形输出
```

`PredictedTrajectoryArray` 才是以后风险评估、动态代价地图、MPPI 等模块应该订阅的正式接口。`MarkerArray` 只是调试画面，不能反过来成为算法输入；否则一旦改颜色、线宽或显示方式，算法接口也会被无意义地破坏。

## 这次 Marker 如何表达预测

每条通过质量门限的轨迹会发出三个同 ID 的 Marker：

| Namespace | 形状 | 含义 |
|---|---|---|
| `prediction_trajectory` | 青色 `LINE_STRIP` | 按时间顺序连接的未来位置 |
| `prediction_points` | 青色 `SPHERE_LIST` | 每个 `0.2 s` 的离散预测点 |
| `prediction_horizon_uncertainty` | 紫色 `LINE_STRIP` 椭圆 | 预测终点的 2σ 位置不确定性 |

紫色范围是真正利用 `Pxx`、`Pxy`、`Pyy` 计算出的二维 2σ 协方差椭圆：椭圆长短轴来自协方差特征值，方向来自 `Pxy`。它不再用半透明实心圆覆盖地图，因此不会遮住青色轨迹。

默认展示模式不会把所有 Track 都画出来：它只显示速度至少 `0.15 m/s`、且按“速度 × 置信度”排序的前 `3` 条预测。静态墙体和大量低质量候选仍在正式 `/dynamic_obstacles/predictions` 消息中，**只是不出现在展示图**。这让调试数据完整、演示画面克制。椭圆半轴还会在视觉上限制到直径 `1.2 m`；这同样只影响 RViz，绝不修改正式协方差。

若想调节展示筛选或椭圆上限，在终端 4 后追加参数，例如：

```bash
ros2 run predictive_nav_prediction prediction_node --ros-args \
  -p use_sim_time:=true \
  -p visualization_min_speed_mps:=0.20 \
  -p visualization_max_trajectories:=2 \
  -p max_visual_uncertainty_diameter_m:=0.8
```

Marker 生命周期设为 `0.35 s`。正常跟踪帧约 `10 Hz`，因此每帧都会刷新；一旦 tracks 停止，仿真时间继续推进时旧图形会自动消失，不会长期残留成假轨迹。

## 编译

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_prediction predictive_nav_bringup
source install/setup.bash
```

这一步给 prediction 包增加了 `visualization_msgs` 依赖，同时把新的 MarkerArray Display 写入 `nav_baseline.rviz`。已经打开的旧 RViz 不会自动读到新配置，关闭后由 launch 重新打开，或手动新增下文的 Display。

## 运行与观察

使用一套干净的终端启动；如果之前异常关闭过 Gazebo，先确认没有残留的 `gz sim`、bridge 和控制脚本。不要同时启动两份 world，否则可能出现多台重名机器人、速度命令正常但物理底盘不动的问题。

按下面顺序打开 **4 个终端**。每个代码块都是完整命令，可以分别直接复制粘贴。

### 终端 1：Gazebo + AMCL + Nav2 + RViz

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

等待 Gazebo、RViz 都打开，且 RViz 左侧 Nav2 的 `Navigation`、`Localization` 变成 `active` 后，再开后面的终端。

### 终端 2：LiDAR 聚类（scan_info_node）

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_perception scan_info_node --ros-args -p use_sim_time:=true
```

### 终端 3：多目标跟踪（tracking_node）

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_tracking tracking_node --ros-args -p use_sim_time:=true
```

### 终端 4：轨迹预测（prediction_node）

```bash
cd ~/PredictiveNav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run predictive_nav_prediction prediction_node --ros-args -p use_sim_time:=true
```

终端 4 启动后应先打印 `prediction time axis`、`track-quality gate` 和 `prediction_node started`。随后收到合格 Track 时会出现 `published predictions`；这时 RViz 才会开始显示青色预测轨迹。

在 RViz 左侧应看到 `Predicted Dynamic Trajectories`。展开它，三个 namespace 均应为勾选状态。默认配置会关闭 `Scan-Derived Clusters (Debug)` 与 `Tracked Obstacles (Debug)`，这是为了得到干净的展示画面；排查感知/关联问题时再手工勾回它们。若旧 RViz 中没有该项：

1. 点击 `Add`；
2. 选择 `MarkerArray`；
3. Topic 设置为 `/dynamic_obstacles/prediction_markers`；
4. Fixed Frame 保持为 `map`（完整导航时）或设为 `odom`（只做 bag 最小回放时）。

可在另一个终端确认发布链路：

```bash
ros2 topic info /dynamic_obstacles/prediction_markers --verbose
ros2 topic echo --once /dynamic_obstacles/predictions
```

正常情况下，第一个命令应看到 publisher 是 `prediction_node`；第二个命令中每条 trajectory 的 `poses` 数量为 `10`，对应 `0.2, 0.4, ..., 2.0 s`。

## 怎样正确解读画面

对一个确实在移动的 Gazebo 动态方块：

- 青色轨迹应从 Track 当前位置向其速度方向延伸；
- 预测越远，紫色终点椭圆通常越大，因为 CV 模型加入了未知加速度过程噪声；
- 方块转弯、停下或被遮挡后，CV 轨迹短暂不准是正常现象：它只假设短时间内速度恒定；
- 青色预测不等于 Gazebo 真值。它是“LiDAR → cluster → tracking → prediction”这一整条估计链的输出。

墙、桌腿等静态物体也可能留下 Track 或预测轨迹。这并不自动说明预测节点错误，而是说明当前 tracker 仍会维护这些可观测目标。此时重点看 `confidence`、速度和轨迹是否合理，不应仅凭颜色把所有 Marker 当成动态障碍物。

## 常见问题

### RViz 没有预测轨迹

按顺序检查：

```bash
ros2 topic info /dynamic_obstacles/tracks
ros2 topic info /dynamic_obstacles/predictions
ros2 topic info /dynamic_obstacles/prediction_markers
```

若 tracks 没有 publisher，先恢复 scan/tracking 链路；若 predictions 有发布但 `trajectories: []`，查看 prediction 终端的 `track quality gate`，表示输入帧有效但没有 Track 达到年龄、置信度、miss 次数和速度等门限；若 markers 有 publisher 而 RViz 不显示，核对 Topic、Fixed Frame、namespace 和 RViz 的 TF 状态。

### 为什么有些 Track 没有显示预测轨迹

这通常是展示筛选，而不是 prediction_node 漏算：默认只有速度不少于 `0.15 m/s`、评分最高的前 3 个候选显示。正式预测 topic 仍包含全部通过质量门限的 Track。若要看所有轨迹，可以把 `visualization_min_speed_mps:=0.0`、`visualization_max_trajectories` 调大；但那应只用于排错，不适合作为展示截图。

### 轨迹突然消失

先看 prediction 节点是否提示 stale input 或 quality gate reject。Marker 只存活 `0.35 s`；它消失通常是在诚实地告诉你当前没有新鲜、合格的预测输入，而不是 RViz 把算法数据删掉。

## 本步完成边界

完成本步后，你能够用图形确认“预测是否沿速度方向传播、预测时间越远不确定性是否增大”，并可保存运行截图作为实验材料。

它仍不是动态避障：Marker 和 prediction_node 都没有订阅或发布控制速度，不能改变 Nav2 或 Gazebo 小车的运动。下一章把预测接入风险评估/局部规划后，才会真正影响避障决策。
