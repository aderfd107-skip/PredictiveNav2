# 01：从 C 到 C++ 的过渡

## 本章完成后，你能做什么

你会先**运行并读懂**一个小型“激光距离预处理”程序，再逐渐理解以后看 ROS 2 C++ 代码时最常见的部分：`std::vector`、`struct`、`class`、构造函数、成员函数、`const`、引用和命名空间。

本章**不会**要求你从空白写代码，也不会要求你背模板元编程、继承体系或复杂智能指针。它们会在项目需要时再引入。

## 先安心：这一章你只要做三件事

第一次学习时，**不要修改任何代码，也不用完成练习**。你只需要：

1. 复制命令并运行已有程序；
2. 对照输出，知道程序做了什么；
3. 按 [跟着我一步一步操作.md](跟着我一步一步操作.md) 的指引，观察一个已经标好的数字变化。

“自己从零写出来”是很后面的目标，不是现在的门槛。你卡在任何一句时，直接问我“第 X 行是什么意思”；我会继续把那一行拆开，不会让你先猜答案。

## 你已经会的 C，和接下来需要的 C++

如果你会下面的 C 代码，就已经有很好的基础：

```c
typedef struct { float x; float y; } Point;
void print_point(const Point *p);
```

对应的 C++ 可以写成：

```cpp
struct Point2D {
  double x{0.0};
  double y{0.0};
};

void print_point(const Point2D& point);
```

这里并没有魔法：`struct` 仍然是装数据的类型；函数仍然接收参数。不同点是：

- `double x{0.0};` 在声明时给安全默认值；
- `const` 表示函数承诺不改这个对象；
- `&` 是引用。它像“原变量的别名”，不复制整个对象，也不像 C 指针那样需要写 `*`、`->` 与空指针判断。

## 第一个程序：模拟清理一帧激光距离

打开 [examples/laser_ranges_intro.cpp](examples/laser_ranges_intro.cpp)。它不依赖 ROS；为了专注 C++，它用一串手工写入的距离模拟 `/scan.ranges`。

打开一个终端，按下面顺序**整行复制**。每复制一行后按一次 Enter；不用理解命令的每个单词。

```bash
cd /home/aderfd/PredictiveNav2
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  cpp_predictive_nav_book/01_C到C++的过渡/examples/laser_ranges_intro.cpp \
  -o /tmp/laser_ranges_intro
/tmp/laser_ranges_intro
```

你应该看见原始距离数量、有效点数量、最近距离和有效距离列表。若命令无输出且返回终端提示符，说明你只编译了程序但没有运行第二行。

## 逐个认识新概念

### 1. `std::vector<double>`：自动管理的一串数

在 C 中，如果有一组距离，常见写法是数组加长度：

```c
double ranges[8] = {0.5, 1.0};
size_t count = 2;
```

数组容量固定，传给函数时还要另传长度。C++ 的 `std::vector<double>` 同时保存元素和长度，并在需要时自动扩容：

```cpp
std::vector<double> valid_ranges;
valid_ranges.push_back(range);
```

`push_back` 意思是“在末尾加入一个元素”。不要自行 `malloc` / `free` 这个 vector；当它离开作用域时，C++ 会自动释放它管理的内存。这种“对象获得资源、对象销毁时自动释放”的思想叫 **RAII**。你现在只要记住：优先用 `std::vector`，不要为普通数组手写内存管理。

### 2. 范围 `for` 循环

```cpp
for (const double range : ranges) {
  // 每次把一个元素的值放入 range
}
```

它相当于“遍历 ranges 中的每一个元素”。这里元素是 `double`，复制成本极低，所以按值取出没问题。将来遍历很大的对象时常写为：

```cpp
for (const Point2D& point : points) {
  // 不复制 point，且承诺不修改它
}
```

读法固定为：`const 类型& 名字` = “只读地、无复制地使用一个已有对象”。

### 3. `struct`：先用它表达纯数据

```cpp
struct ScanSummary {
  std::size_t valid_count{0};
  double nearest_range{0.0};
  std::vector<double> valid_ranges;
};
```

这个结构将一帧处理结果打包在一起。`std::size_t` 是表示容器长度和下标的无符号整数类型；`vector.size()` 的返回类型正是它。

在本项目中，后续会有类似的“数据包”：一个 `Cluster` 保存质心、尺寸和点数；一个 `Track` 保存 ID、位置、速度、协方差。

### 4. `class`：把状态和操作放在一起

例子中的 `RangeFilter` 保存两个规则：最小有效距离和最大有效距离。它也提供一个 `filter()` 操作。

```cpp
class RangeFilter {
 public:
  RangeFilter(double min_range, double max_range);
  ScanSummary filter(const std::vector<double>& ranges) const;

 private:
  double min_range_;
  double max_range_;
};
```

- `public` 后的成员是类外可使用的接口；
- `private` 后的成员只能由该类内部使用，外部不能随意把阈值改成非法值；
- `RangeFilter(...)` 与类同名，它是**构造函数**，对象创建时自动执行；
- 函数末尾的 `const` 表示此函数不修改对象自身的成员。它不是“返回值是常量”。

`min_range_` 的末尾下划线是本项目采用的成员变量命名习惯。它帮助你一眼区分参数 `min_range` 和成员 `min_range_`。

### 5. 引用 `&` 与指针 `*`

在 C 中你常用指针让函数读取或修改调用者的数据。C++ 仍然支持指针，但普通“必定存在的输入参数”常用引用：

```cpp
ScanSummary filter(const std::vector<double>& ranges) const;
```

它表示：

- `ranges` 是调用者拥有的 vector；
- 函数不复制整串数据；
- 函数不修改它；
- 调用方必须传入一个真实 vector，不能传“没有对象”的空指针。

什么时候仍用指针？当“没有对象”本身是合法含义时，例如一个可选资源、可能为空的接口。第 2 章遇到 ROS 2 的共享指针时会专门解释；目前不要把引用和指针混用。

### 6. `namespace`：避免名字冲突

例子包在下面的命名空间中：

```cpp
namespace predictive_nav::tutorial {
// 代码放在这里
}
```

这不是创建目录，而是给名字加前缀。完整名字其实是 `predictive_nav::tutorial::RangeFilter`。将来项目里有多个包和很多 `Track`、`Point` 等常见名字，命名空间可避免冲突。

## 先读懂程序，而不是记住每个符号

你暂时不需要回答题目。先把下面四句话当作“读程序时的路线图”：

1. 输入是什么？一串模拟激光距离。
2. 谁处理它？保存阈值的 `RangeFilter` 对象。
3. 什么数据被丢弃？非有限数字，以及不在最小/最大距离范围内的数。
4. 输出是什么？`ScanSummary`，包含有效数量、最近距离和有效距离。

等你对这四句话熟悉后，再打开 [代码逐行讲解.md](代码逐行讲解.md)。它会从第 1 行开始解释这份程序；一次只读一个小节即可。

## 与 ROS 2 的连接

真正的 ROS 2 版本会把手工的 `ranges` 替换为 `sensor_msgs::msg::LaserScan::ranges`，并会增加时间戳、角度和 TF 处理。但“读取一组距离 → 过滤无效值 → 返回一个结果”的 C++ 结构完全相同。

第 5 章将把本章的思路扩展为真正的 `/scan` 预处理节点。现在先让这套基础写法变得自然。

## 常见错误

| 现象 | 原因与处理 |
| --- | --- |
| `g++: command not found` | 系统没有 C++ 编译器；先安装构建工具后再继续。不要跳过编译。 |
| 编译命令中的路径找不到 | 确认当前目录是仓库根目录，而不是 `cpp_predictive_nav_book/01...` 目录。 |
| `std::vector` 未声明 | 忘记 `#include <vector>`。每个文件应直接包含自己使用的标准库头文件。 |
| 把 `const` 函数改为修改成员后无法编译 | 这是编译器在保护你：去掉 `const` 前先确认函数确实应改变对象状态。 |
| 以为 `&` 一定等于“取地址” | C++ 中声明里的 `Type&` 是引用；表达式里的 `&value` 才是取地址。 |

## 本章检查表

- [ ] 我成功编译并运行了 `laser_ranges_intro.cpp`。
- [ ] 我能说出 `std::vector` 比 C 固定数组方便在哪里。
- [ ] 我能读懂 `const std::vector<double>&`。
- [ ] 我知道构造函数在创建对象时执行。
- [ ] 我能解释 `public` 与 `private` 的区别。
- [ ] 我完成了 [跟着我一步一步操作.md](跟着我一步一步操作.md) 的前三步。
- [ ] 我知道现阶段不用从空白写代码。

下一章：[02_ROS2_C++节点入门](../02_ROS2_C++节点入门/README.md)。
