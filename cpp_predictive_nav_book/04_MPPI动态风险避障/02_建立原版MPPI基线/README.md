# 02：建立原版 MPPI 基线

在没有自定义风险模块时，先让 Nav2 原版 MPPI 在同一地图和动态场景中稳定运行、记录结果。

**基线**是之后比较的参照物。没有它，就无法证明自定义 critic 改善了什么，也无法区分是 MPPI 配置问题还是新代码问题。

完成标准：保存一套独立 MPPI 参数和一次可复现的导航结果，不接入 predictions。

## 本步是否写代码

不写 C++ 算法，但你要亲手写配置并完成一次真实运行。这是低代码、高价值的工程步骤：以后所有“动态风险带来的改善”都依赖这份对照组。

## 你要亲自写的文件

新建一份独立配置，例如：

```text
src/predictive_nav_bringup/config/nav2_mppi_baseline.yaml
```

不要直接覆盖现有 `nav2_dwb.yaml`。你需要自己填写并解释：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      motion_model: "diff_drive"
    diff_drive:
      plugin: "nav2_mppi_controller::DiffDriveMotionModel"
      # 后续补充 time_steps、model_dt、batch_size、速度和加速度约束、critics
```

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
