# 02：创建真正的 C++ 包

## 这一小步的目标

我们已经确认了项目有真实的 `/scan` 输入。现在要在 `src/` 中建立一个**真正的 ROS 2 C++ 包**，名字是：

```text
predictive_nav_perception
```

以后所有“从 `/scan` 识别障碍物”的 C++ 代码都会放在这个包里。它不是教材里的临时练习，也不是复制后再移植的副本；它就是项目源码。

这一小步结束时，包里还没有节点代码。这是故意的：先确认“房子地基能构建”，下一步才往里面添加第一个节点。

## 我已经为你创建的文件

项目当前新增了下面的目录和文件：

```text
src/predictive_nav_perception/
├── package.xml
├── CMakeLists.txt
├── include/predictive_nav_perception/   # 以后放 .hpp 头文件
└── src/                                 # 以后放 .cpp 源文件
```

你现在不需要手动创建、也不需要自己填写这两个配置文件。先阅读“它们各自负责什么”，再执行构建命令确认它们没有问题。

## 为什么一个 C++ ROS 2 包不只是 `.cpp` 文件

普通 C++ 程序常常只有一个 `.cpp` 文件和一个编译命令。ROS 2 项目是多个包一起工作，所以需要两个“说明文件”：

```text
package.xml     告诉 ROS 2：我是谁、我的依赖是什么。
CMakeLists.txt  告诉构建工具：怎样编译和安装我的 C++ 代码。
```

以后你每新增一个 C++ 节点，通常都要同时：

1. 创建或修改 `.cpp` 源文件；
2. 在 `CMakeLists.txt` 登记“把它编译为可运行节点”；
3. 在 `package.xml` 登记它使用的 ROS 2 依赖。

现在只完成了第 0 步：让 ROS 2 认识这个包本身。

## 看懂 `package.xml`

打开：

```text
/home/aderfd/PredictiveNav2/src/predictive_nav_perception/package.xml
```

目前只需要认识这几行：

```xml
<name>predictive_nav_perception</name>
<buildtool_depend>ament_cmake</buildtool_depend>
```

第一行是包的唯一名字。ROS 2 命令、依赖配置和 CMake 工程名都会使用它，因此统一用小写字母和下划线。

第二行表示：这个包用 `ament_cmake` 构建。`ament_cmake` 是 ROS 2 在 CMake 基础上提供的一套构建规则。

现在文件中还没有 `rclcpp`、`sensor_msgs` 等依赖，因为当前包还没有用到它们。下一步创建订阅 `/scan` 的节点时，再只添加真正用到的依赖；这样每一次新增内容都有明确理由。

## 看懂 `CMakeLists.txt`

打开：

```text
/home/aderfd/PredictiveNav2/src/predictive_nav_perception/CMakeLists.txt
```

最重要的三部分是：

```cmake
project(predictive_nav_perception)
find_package(ament_cmake REQUIRED)
ament_package()
```

可以先按中文记住：

- `project(...)`：本 CMake 工程的名字；必须与 `package.xml` 中的包名一致；
- `find_package(...)`：我要使用 `ament_cmake`，构建时找不到它就报错；
- `ament_package()`：把这里登记为一个 ROS 2 包。没有它，`colcon` 不会把这里正确处理成 ROS 2 ament 包。

`cmake_minimum_required(VERSION 3.8)` 是要求构建时的 CMake 不低于该版本；现在不需要改它。

## 你现在要做的操作：构建这个真实包

打开一个新终端（或确认没有 Gazebo 仍在占用同一个终端），逐行复制：

```bash
cd /home/aderfd/PredictiveNav2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select predictive_nav_perception
```

构建成功时，末尾应出现类似：

```text
Summary: 1 package finished
```

这时 C++ 代码还不存在，为什么要构建？因为我们验证的是：包名、XML 格式、CMake 规则和 ROS 2 构建系统之间已经能正确协作。以后出现问题时，就能确认是“新加的节点代码”而不是“包骨架”造成的。

## 不要担心 `build/`、`install/`、`log/`

`colcon build` 会在仓库根目录自动创建或更新这三个目录：

```text
build/    构建过程的中间文件
install/  构建成功后可被 ROS 2 运行的安装结果
log/      构建日志
```

它们不是你要手写的源码，也不需要逐个看懂。我们真正维护的源码仍然是 `src/predictive_nav_perception/` 下的文件。

## 遇到问题怎么办

### `colcon: command not found`

先确认已经执行：

```bash
source /opt/ros/jazzy/setup.bash
```

若仍失败，把完整输出发给我；不要自行安装或删除软件包。

### `Package 'predictive_nav_perception' not found`

确认你执行构建命令前已经进入仓库根目录：

```bash
cd /home/aderfd/PredictiveNav2
```

然后重新构建。`src/predictive_nav_perception/package.xml` 必须存在；它就是 `colcon` 找包的标志。

### XML 或 CMake 报错

不要自己猜着删改。把从第一条 `Error` 开始到构建结束的完整文本发给我。当前包很小，错误通常能很快定位。

## 这一小步的完成标准

- [ ] 我知道 `predictive_nav_perception` 是项目真实源码包，而不是教材例子。
- [ ] 我能找到它的 `package.xml` 和 `CMakeLists.txt`。
- [ ] 我知道 `package.xml` 管依赖与身份，`CMakeLists.txt` 管构建方式。
- [ ] `colcon build --packages-select predictive_nav_perception` 成功，末尾显示 1 个包完成。

完成后告诉我构建末尾的输出。下一步才会在 `src/predictive_nav_perception/src/` 中添加第一份真正的 C++ 文件：订阅 `/scan` 并打印它的基本信息。

## 本章关联的 ROS 2 知识

**本章必须懂**：一个 ROS 2 C++ 包至少要让系统知道“它叫什么、依赖什么、怎样构建”。`package.xml` 负责前两件事，`CMakeLists.txt` 负责最后一件事；构建完成后重新 `source install/setup.bash`，当前终端才能找到新程序。

**可选扩展**：不用现在研究 ament 宏、overlay workspace 或发行包规则；遇到构建错误时再针对性补。
