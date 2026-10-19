# 02：建立原版 MPPI 基线

> **本步目标：** 让原版 Nav2 MPPI 在当前地图中可重复运行，作为以后动态风险模块的对照组；本步不写自定义 C++。
>
> **AI 会做什么：** 生成 `nav2_mppi_baseline.yaml`，并根据本项目的差速底盘限制说明每个参数的起始值。
>
> **你只需要做到：** 能解释 `FollowPath` 是控制器实例名、`plugin` 是实现类、`motion_model` 是候选轨迹运动学模型，并实际启动一次。

在没有自定义风险模块时，先让 Nav2 原版 MPPI 在同一地图和动态场景中稳定运行、记录结果。

**基线**是之后比较的参照物。没有它，就无法证明自定义 critic 改善了什么，也无法区分是 MPPI 配置问题还是新代码问题。

完成标准：保存一套独立 MPPI 参数和一次可复现的导航结果，不接入 predictions。

## 本步是否写代码

不写 C++ 算法。AI 可生成配置；你要审读速度、加速度、预测窗和 critic 的含义，并完成一次真实运行。这是低代码、高价值的工程步骤：以后所有“动态风险带来的改善”都依赖这份对照组。

## AI 生成、你要审读的文件

新建一份独立配置，例如：

```text
src/predictive_nav_bringup/config/nav2_mppi_baseline.yaml
```

不要直接覆盖现有 `nav2_dwb.yaml`。让 AI 生成并解释：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # Jazzy 的合法值是 "DiffDrive"、"Omni"、"Ackermann"。
      # 本项目是差速底盘，因此选 DiffDrive。
      motion_model: "DiffDrive"
      # 后续补充 time_steps、model_dt、batch_size、速度和加速度约束、critics
```

> 不要在 Jazzy 配置中再写 `diff_drive:` 与 `diff_drive.plugin`。`DiffDriveMotionModel` 是 Nav2 内部 C++ 类名；Jazzy 用上面的 `motion_model: "DiffDrive"` 选择它。那种嵌套 plugin 写法属于其他 Nav2 版本，复制过来会造成参数不生效或启动失败。

具体参数名会在实际启动前根据本机 Jazzy MPPI 文档和报错逐项确认；不要从网络随机复制一整份其他 ROS 版本的 YAML。

## 必须理解的参数

| 参数 | 你要回答的问题 |
|---|---|
| `time_steps × model_dt` | MPPI 向前看多长时间？它为什么应与第 03 章预测 horizon 可比较？ |
| `batch_size` | 每周期比较多少条候选？增大为何更慢？ |
| `vx_max`、`wz_max`、加速度限制 | 为什么必须与差速底盘和真机能力一致？ |
| `critics` | 原版 MPPI 评分规则有哪些？此时为什么不能出现 `DynamicRiskCritic`？ |

## 必须保存的证据

- 原版 MPPI 的 YAML；
- 启动命令、地图、起点、目标、动态 actor 开关；
- 一次成功和一次失败/需要调参的运行记录；
- `/cmd_vel`、action status 与 rosbag 名称。

本步没有任何 `#include` 或新函数；它的产物是可信 baseline，而不是“配置碰巧能跑”。

## 生成、运行与验证

先不要改 `nav2_dwb.yaml`。复制它的目的只是保留 AMCL、costmap、行为树、速度限制等相同条件；随后只替换 `controller_server` 里的控制器配置。

```bash
cd ~/PredictiveNav2
cp src/predictive_nav_bringup/config/nav2_dwb.yaml \
  src/predictive_nav_bringup/config/nav2_mppi_baseline.yaml
```

然后让 AI 在新文件中完成下面三件事，并生成中文注释：

1. 将 `FollowPath.plugin` 改为 `nav2_mppi_controller::MPPIController`；
2. 删除 DWB 专用参数，例如 `vx_samples`、`vtheta_samples`、`sim_time` 和 DWB critic 列表；
3. 写入 MPPI 的运动模型、预测步数、时间步长、速度/加速度约束和原版 critics。

先不要凭记忆补参数。你需要先在本机查可用项：

```bash
ros2 pkg prefix nav2_mppi_controller
ros2 param list /controller_server | rg -i 'mppi|followpath'
```

第二条命令要在 MPPI controller 已启动后执行；如果还未启动，先记录“当前无此参数”，不要把它误认为报错。

## 你应先写出的配置草稿

下面是 AI 生成配置时必须保留的结构。每个参数都要有单位和选择理由；你要能解释这些理由，而不必从零填写 `TODO`。

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      motion_model: "DiffDrive"
      time_steps: TODO            # 个
      model_dt: TODO              # s
      batch_size: TODO            # 条候选
      vx_max: TODO                # m/s，不能超过现有 0.35
      wz_max: TODO                # rad/s
      critics: [TODO]
```

让 AI 完成 YAML 草稿、写出每个 TODO 的理由；你逐项审读，再核对 Jazzy 的准确 critic 名与参数组合。这样不会因为复制不同 ROS 版本的配置而浪费时间。

## 本节通过条件

- DWB 配置文件未被修改；
- 新 YAML 被 launch 实际加载；
- 没有 `DynamicRiskCritic`；
- 对静态场景能发送并完成一个导航目标；
- 记录一次动态方块横穿时的原版 MPPI 行为，无论成功或失败。
