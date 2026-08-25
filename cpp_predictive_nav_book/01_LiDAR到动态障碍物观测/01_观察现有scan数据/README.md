# 01：先观察现有的 `/scan` 数据

## 这一小步的目标

这一小步**不写 C++，不修改项目任何文件**。

你只需要亲眼确认三件事：

1. 现有 Gazebo 仿真正在发布 `/scan`；
2. `/scan` 的类型确实是 `sensor_msgs/msg/LaserScan`；
3. 它里面最重要的数据是一串距离 `ranges`，这正是以后 C++ 感知节点的输入。

如果没有先看过真实输入，直接写“激光处理代码”只会变成背代码。现在先建立直觉：**传感器给程序的不是一张图片，也不是“障碍物列表”，而是一圈距离数字。**

## 你会看到的完整链路

```text
Gazebo 中的机器人 LiDAR
        ↓
Gazebo 话题 /nav_robot/scan
        ↓  （ros_gz_bridge 桥接并改名）
ROS 2 话题 /scan
        ↓
本步骤：你用命令观察它
        ↓
后续步骤：你写的 C++ 节点订阅它
```

现在不要试图理解桥接程序、URDF 或 Python 动态 actor 的代码；那些不是这一小步的任务。

## 准备：打开两个终端

按 **Ctrl + Alt + T** 打开第一个终端，称为“终端 A”。它负责启动仿真。

再按一次 **Ctrl + Alt + T** 打开第二个终端，称为“终端 B”。它负责观察话题。

两个终端需要同时开着：仿真运行时，另一个终端才能看到它发布的数据。

## 第 1 步：在终端 A 启动动态场景

在终端 A 中，逐行复制以下命令；每行复制后按 Enter：

```bash
source /opt/ros/jazzy/setup.bash
source /home/aderfd/PredictiveNav2/install/setup.bash
ros2 launch predictive_nav_bringup nav_baseline.launch.py enable_dynamic_obstacle:=true
```

现在等待 Gazebo 和 RViz 打开。第一次启动可能需要十几秒；终端 A 会不断输出日志，这是正常的。

这条启动命令做了以下事情，你暂时只要有印象：

- 启动 Gazebo 室内场景和机器人；
- 启动三个会来回移动的方块障碍物；
- 把 Gazebo 的激光数据桥接为 ROS 2 的 `/scan`；
- 启动已有的 AMCL、Nav2 和 RViz。

**不要关闭终端 A。** 如果按 Ctrl + C，所有仿真和 `/scan` 都会停止。

## 第 2 步：在终端 B 确认 `/scan` 存在

在终端 B 中，同样逐行复制：

```bash
source /opt/ros/jazzy/setup.bash
source /home/aderfd/PredictiveNav2/install/setup.bash
ros2 topic list
```

屏幕会列出许多话题。你现在只需在其中找到：

```text
/scan
```

如果列表中有 `/scan`，说明仿真 LiDAR 的 ROS 2 数据通道已经通了。

接着复制：

```bash
ros2 topic info /scan -v
```

在输出中找到类型。应当是：

```text
Type: sensor_msgs/msg/LaserScan
```

其他 `Publisher count`、QoS 等文字现在不用理解；后面真正写订阅节点时会只讲与你需要的部分。

## 第 3 步：看一次真实消息

仍在终端 B，复制：

```bash
ros2 topic echo /scan --once
```

它会显示一整条真实的激光消息，然后自动停止。内容较长是正常的；你只找下面四部分，不用逐个读所有数字：

```text
header:
  frame_id: lidar_link
angle_min: ...
angle_increment: ...
ranges:
- ...
- ...
```

它们的当前含义是：

| 字段 | 先这样理解 |
| --- | --- |
| `header.stamp` | 这一圈激光是什么时刻测到的。以后 TF 转换必须使用这个时间。 |
| `header.frame_id: lidar_link` | 数字最初是以机器人 LiDAR 自己为原点表达的。 |
| `angle_min` | `ranges` 第一个数字对应的起始方向。 |
| `angle_increment` | 列表中相邻两个数字的方向间隔。 |
| `ranges` | 每个方向实际测到的距离，单位是米。它是后续 C++ 节点最直接的输入。 |

例如，如果某个 `ranges` 数字是 `2.0`，它只表示“激光朝对应方向 2 米处撞到了东西”；它还没有说那个东西是墙、桌子还是动态方块。

## 第 4 步：确认它持续发布

复制：

```bash
ros2 topic hz /scan --use-sim-time
```

等待约 5 秒后按 **Ctrl + C** 停止观察。你应看到类似：

```text
average rate: 10.0
```

数值不必完全等于 10，但正常情况下应接近 10 Hz。Hz 读作“赫兹”，意思是每秒发布多少条消息。这里约 10 Hz 就是激光每秒大约更新 10 次。

这里特意加了 `--use-sim-time`，意思是“按 Gazebo 的仿真时间统计”，而不是按电脑现实世界的时间统计。本项目 LiDAR 在仿真中配置为每秒更新 10 次；但如果电脑的 CPU 或软件渲染较忙，Gazebo 可能以低于真实时间的速度运行。此时不加该参数，常会看到例如 `3.8 Hz`，它反映的是仿真在真实世界中跑慢了，并不代表 LiDAR 的仿真更新配置被改成了 3.8 Hz。

为什么要看频率？后面做跟踪时，需要知道两帧间隔大约多久，才能正确估计障碍物速度；如果话题根本没在持续更新，后面的算法再正确也没有输入。以后真正写跟踪时，我们会使用消息自己的时间戳，而不是假设电脑每秒总能收到 10 条消息。

## 你现在已经完成了什么

你还没有写任何 C++，但已经确认了未来第一个 C++ 节点的真实输入：

```text
节点订阅：/scan
消息类型：sensor_msgs/msg/LaserScan
关键内容：ranges、角度、时间戳、lidar_link 坐标系
更新速度：约 10 Hz
```

这不是额外步骤，而是写代码前必要的“确认接口”。后续 C++ 代码不应凭空猜测话题名、消息类型或数据格式。

## 遇到问题怎么办

### `/scan` 不在话题列表中

先确认终端 A 的仿真窗口还开着，并且启动命令没有报错退出。然后在终端 B 重新执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/aderfd/PredictiveNav2/install/setup.bash
ros2 topic list
```

如果仍没有 `/scan`，不要自行改 launch 文件；把终端 A 中最早出现的红色错误信息，以及终端 B 的输出发给我。

### `ros2: command not found`

说明当前终端没有加载 ROS 2 环境。重新执行：

```bash
source /opt/ros/jazzy/setup.bash
```

### `Package 'predictive_nav_bringup' not found`

说明项目安装空间尚未构建，或没有加载。先在一个新终端执行：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_description predictive_nav_simulation predictive_nav_bringup
```

构建成功后，再回到终端 A，从本章第 1 步重新开始。

### 不加 `--use-sim-time` 时只有约 3～4 Hz

这通常不是错误。它表示 Gazebo 当前约以 0.3～0.4 倍实时速度运行；软件渲染、打开 Gazebo/RViz 和计算机 CPU 负载都会影响它。请改用本章第 4 步的命令：

```bash
ros2 topic hz /scan --use-sim-time
```

第一模块目前只要求 `/scan` 持续到达，不要求你先把仿真加速。若加上 `--use-sim-time` 后仍长期远低于 10 Hz，请把该输出发给我；后续会单独评估 Gazebo 性能和桥接情况。

### `ros2 topic echo /scan --once` 一直没有输出

这表示 `/scan` 虽然可能被列出，但目前没有新消息到达。确认 Gazebo 没有暂停、终端 A 仍在运行，并把 `ros2 topic info /scan -v` 的完整输出发给我。

## 这一小步的完成标准

- [ ] Gazebo/RViz 已启动，且动态方块正在移动。
- [ ] `ros2 topic list` 中出现 `/scan`。
- [ ] `ros2 topic info /scan -v` 显示 `sensor_msgs/msg/LaserScan`。
- [ ] `ros2 topic echo /scan --once` 中看到了 `frame_id`、角度和 `ranges`。
- [ ] `ros2 topic hz /scan --use-sim-time` 显示稳定的持续更新（正常时接近 10 Hz）。

完成后先告诉我你在哪一步看到了什么；下一步才会创建第一个真实的 C++ 包。你无需提前进入 `02_创建真实C++包`。

## 本章关联的 ROS 2 知识

**本章必须懂**：消息的 `header.frame_id` 说明数据在哪个坐标系表达；`header.stamp` 说明它何时测得。`ros2 topic hz` 默认按现实墙钟计时，而 `--use-sim-time` 按 Gazebo 的仿真时钟计时，所以你已经观察到两种 rate 不同。

**可选扩展**：暂时不必研究 ROS clock 的源码或所有时间类型；只要后续涉及仿真数据时记得检查是否使用 sim time。
