# inspection_core 代码完全讲解

> 本文档面向新手，逐行讲解 `inspection_core` 包的每一个文件、每一个函数、每一个设计决策。
> 读完本文后，你应该能够在面试中清楚地解释：**"我用 A* 算法实现了一个多机器人路径规划系统的核心模块"**。

---

## 目录

1. [这个包是干什么的](#1-这个包是干什么的)
2. [整体架构](#2-整体架构)
3. [grid_graph.py — 地图模块](#3-grid_graphpy--地图模块)
4. [astar.py — 路径搜索模块](#4-astarpy--路径搜索模块)
5. [offline_demo.py — 离线演示](#5-offline_demopy--离线演示)
6. [test_astar.py — 单元测试](#6-test_astarpy--单元测试)
7. [面试如何讲](#7-面试如何讲)

---

## 1. 这个包是干什么的

### 一句话

**输入**一张已知地图 + 起点坐标 + 目标点坐标 → **输出**一条最短的无碰撞路径。

### 在整个项目中的位置

```
┌──────────────────────────────────────────────┐
│  任务分配层（未来）                              │
│  "哪个机器人做哪个任务"                           │
├──────────────────────────────────────────────┤
│  路径规划层 ← inspection_core 在这里             │
│  "从 A 到 B 怎么走才不撞墙"                       │
├──────────────────────────────────────────────┤
│  执行层（未来）                                  │
│  "把路径发给 Nav2，驱动机器人实际走"               │
└──────────────────────────────────────────────┘
```

### 为什么不直接调 Nav2

Nav2 能做的事：接收一个目标点，驱动机器人开过去。

Nav2 不能做的事：
- 多机器人冲突检测（两台机器人在走廊相遇怎么办）
- 全局任务分配（三个任务分配给两台机器人，谁做哪个）
- 时空路径规划（A 在 t=5s 时经过的位置，B 必须避开）

**inspection_core 提供了 Nav2 之上的"大脑"**：在地图的栅格上跑自己的算法，输出 waypoint，再交给 Nav2 执行。

---

## 2. 整体架构

### 文件结构

```
src/inspection_core/
├── CMakeLists.txt              # ROS 2 编译配置
├── package.xml                 # 包元信息（名字、依赖）
├── inspection_core/
│   ├── __init__.py             # 包初始化（空文件，标识这是一个 Python 包）
│   ├── grid_graph.py           # ① 地图模块：加载 PGM、坐标转换、障碍物膨胀
│   ├── astar.py                # ② A* 搜索 + 路径转换
│   └── offline_demo.py         # ③ 离线演示脚本
└── test/
    └── test_astar.py           # ④ 单元测试（17 个测试用例）
```

### 模块依赖关系

```
grid_graph.py  （底层：提供地图数据结构）
     ↑
astar.py       （中层：在图上搜索路径）
     ↑
offline_demo.py（上层：加载真实地图，跑 A*，输出结果）
```

`astar.py` 不依赖 ROS，`grid_graph.py` 不依赖 ROS ——这是刻意设计的。纯 Python 代码可以脱离 ROS 环境运行和测试，速度快，调试方便。

---

## 3. grid_graph.py — 地图模块

这个文件是整个包的基础。它做四件事：
1. 读取 PGM 图片 + YAML 元数据
2. 把像素值转换成 `free / occupied / unknown` 三种状态
3. 提供世界坐标 ↔ 栅格坐标的转换
4. 提供障碍物膨胀（inflation）

### 3.1 为什么用栅格地图

真实世界是连续的（小数点后无限位），计算机只能处理离散的。把地图切成小格子，每个格子 0.05m × 0.05m：

```
连续世界                          离散栅格
┌──────────────────┐            ┌──┬──┬──┬──┐
│                  │            │  │  │  │  │
│   机器人→ ●      │    →       │  │  │● │  │  每个格子 = 5cm×5cm
│                  │            │  │  │  │  │
│  ██████████      │            │  │  │██│██│  █ = 障碍物格子
└──────────────────┘            └──┴──┴──┴──┘
```

14m × 10m 的地图 ÷ 0.05m = 280 × 200 个格子 = 56,000 个 cell。

### 3.2 PGM 图片格式

PGM（Portable GrayMap）是最简单的图片格式之一，每个像素用一个数字表示灰度：

```
P5                    ← 魔数，表示二进制 PGM
280 200               ← 宽 280，高 200
255                   ← 最大灰度值（0=黑, 255=白）
□□□□□□□□□□□□□□...     ← 56000 字节的像素数据（280×200）
```

建图时 `map_saver` 保存的 PGM 只用三种灰度值：

| 像素值 | 颜色 | 含义 |
|--------|------|------|
| 0 | 黑色 | 墙壁/障碍物 |
| 205 | 灰色 | 未知区域（激光没扫到） |
| 254 | 白色 | 可通行空地 |

### 3.3 代码逐段讲解

#### 类型定义

```python
Cell = Tuple[int, int]        # (列, 行)，如 (140, 99)
WorldPoint = Tuple[float, float]  # (x, y) 单位米，如 (0.0, 0.0)
```

设计意图：用类型别名让代码更可读。`Cell` 和 `WorldPoint` 本质上都是 tuple，但名字告诉你它在哪个坐标系。

#### YAML 元数据文件

```yaml
image: office_service_mvp.pgm    # 图片文件名
mode: trinary                     # 三值模式
resolution: 0.050                 # 每个像素 = 0.05 米
origin: [-7.011, -4.993, 0]      # 图片左下角像素在世界坐标系中的位置
negate: 0                         # 0 = 白为空地，黑为墙
occupied_thresh: 0.65             # 灰度值 > 65% → 空地
free_thresh: 0.196                # 灰度值 < 19.6% → 障碍物
```

**origin 是最容易搞混的概念**。想象把 PGM 图片放在世界坐标系中：

```
世界坐标系原点 (0,0)
    │
    │
    ├──── origin (-7.011, -4.993)
    │     ┌────────────────────┐
    │     │     PGM 图片       │  宽 280×0.05=14m
    │     │     覆盖范围：     │  高 200×0.05=10m
    │     │  x: -7.0 ~ +7.0   │
    │     │  y: -5.0 ~ +5.0   │
    │     └────────────────────┘
    │
```

origin 是**图片左下角那个像素的中心**在世界坐标系中的位置。

#### 坐标转换

```python
def world_to_cell(self, x: float, y: float) -> Cell:
    col = int((x - self._origin[0]) / self._resolution)
    row = int((y - self._origin[1]) / self._resolution)
    return (col, row)
```

举例：世界坐标 (5.8, -3.85) 在图片中的位置？

```
col = (5.8 - (-7.011)) / 0.05 = 12.811 / 0.05 = 256.22 → int = 256
row = (-3.85 - (-4.993)) / 0.05 = 1.143 / 0.05 = 22.86 → int = 22
结果: (256, 22)
```

反过来：

```python
def cell_to_world(self, col: int, row: int) -> WorldPoint:
    x = self._origin[0] + (col + 0.5) * self._resolution
    y = self._origin[1] + (row + 0.5) * self._resolution
    return (x, y)
```

`+ 0.5` 是因为返回的是格子的**中心点**坐标，不是左下角。

#### PGM 读取与阈值判断

```python
pct = val / max_val    # 归一化到 0~1
if pct <= self._free_thresh:       # pct < 0.196 → 深色像素
    occ = OCC_OCCUPIED              # → 障碍物
elif pct >= self._occupied_thresh: # pct > 0.65 → 浅色像素
    occ = OCC_FREE                  # → 空地
else:                              # 0.196 ~ 0.65 之间
    occ = OCC_UNKNOWN               # → 未知
```

关键理解：**深色像素（接近 0）= 障碍物，浅色像素（接近 255）= 空地**。这和人的直觉一致——白纸黑字，黑色是墙。

#### 障碍物膨胀（inflation）

```python
def _inflate(self, radius_cells: int) -> List[List[int]]:
```

为什么需要膨胀？机器人不是一个点，它有体积（0.62m × 0.46m）。如果把机器人当质点处理，规划出来的路径可能让机器人擦墙。

解决方法：**把障碍物"变胖"，然后把机器人当质点**。

```
膨胀前:                         膨胀后:
┌──────────┐                   ┌──────────────┐
│  真实墙壁  │        →         │   ██████████  │  墙壁向外扩大了
│  ██████   │                   │   ██████████  │  3 个格子 (≈0.15m)
│           │                   │   ██      ██  │
│  ● 路径   │                   │   ██  ●   ██  │  路径自动远离
└──────────┘                   └──────────────┘
```

代码逻辑：遍历每个格子，如果是障碍物，把它周围 `radius_cells` 范围内的格子也标记为障碍物。

#### 邻居查询

```python
_NEIGHBOUR_OFFSETS = [(0,1), (1,0), (0,-1), (-1,0)]  # 上右下左

def neighbours(self, col, row):
    for dc, dr in self._NEIGHBOUR_OFFSETS:
        nc, nr = col + dc, row + dr
        if self.is_free(nc, nr):
            yield (nc, nr)
```

**为什么用 4 邻接而不是 8 邻接？**

```
4 邻接（上下左右）:        8 邻接（加对角线）:
    ■                        ■ ■ ■
  ■ ● ■                      ■ ● ■
    ■                        ■ ■ ■
```

选择 4 邻接的原因：
- 后续做多机器人冲突检测时，**边冲突**更容易定义（A 从 a→b，B 从 b→a 在同一时刻 = 冲突）
- 8 邻接的对角线移动会穿过墙角和窄通道，导致路径不安全
- Manhattan 距离是 4 邻接的**可采纳启发函数**（永远不会高估）

#### snap_to_free — 自动吸附

```python
def snap_to_free(self, col, row, max_radius=30):
```

问题场景：你给的目标坐标恰好落在障碍物上（因为 SLAM 噪点或 inflation）。

解决方法：BFS（广度优先搜索）从目标点出发，一圈一圈向外找最近的 free cell。

```
搜索半径 = 0:  █ ← 目标点，occupied
搜索半径 = 1:  █ █ □     □ = 找到！吸附到这里
               █ █ □
搜索半径 = 2:  ...
```

#### 启发函数

```python
def heuristic(self, a: Cell, b: Cell) -> float:
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))
```

Manhattan 距离 = 横向格数 + 纵向格数。对于 4 邻接网格，这个启发函数是**可采纳的**——永远不会高估实际距离，保证 A* 找到的是最优路径。

---

## 4. astar.py — 路径搜索模块

### 4.1 A* 算法原理

A* 是 Dijkstra 的改进版。Dijkstra 向所有方向均匀扩散，A* 用启发函数把搜索"拉向"目标方向。

```
Dijkstra（无方向）:                A*（有方向）:
从起点均匀扩散                     从起点向目标方向优先扩散
     ░░░░░░░░░░                      ············
     ░░░░░░░░░░                      ··░░░░░░░░·
     ░░░░●░░░░░                      ··░░░●░░░░·
     ░░░░░░░░░░                      ··░░░░░░░░·
     ░░░░░░░░░░                      ····★······
     ░░░░░░░░★░                                              
                                        ★ = 目标
                                        ● = 起点
                                        ░ = 搜索过的节点
                                        · = 未搜索的节点
```

A* 搜索的节点数量远少于 Dijkstra，但结果仍然是**最优的**（只要启发函数可采纳）。

### 4.2 代码逐段讲解

#### 数据结构

```python
g_score: Dict[Cell, float] = {start: 0.0}   # 起点到每个节点的实际代价
came_from: Dict[Cell, Cell] = {}             # 每个节点是从哪里来的
open_set: List[Tuple[float, int, Cell]] = [] # 优先队列（最小堆）
```

**g_score** 和 **f_score** 的区别：

- `g(n)` = 从起点到节点 n 的**实际代价**（走了多少步）
- `h(n)` = 从节点 n 到目标的**估计代价**（Manhattan 距离）
- `f(n)` = g(n) + h(n) = 经过 n 到目标的**估计总代价**

#### 主循环

```python
while open_set:
    f_val, _, current = heapq.heappop(open_set)  # 取 f 值最小的节点
    
    if current == goal:
        return _reconstruct_path(came_from, current)  # 找到了！
    
    for nb in graph.neighbours(*current):       # 遍历四个邻居
        tentative_g = g_score[current] + 1      # 每步代价 = 1
        
        if tentative_g < g_score.get(nb, inf):  # 发现更短路径
            came_from[nb] = current              # 记录来路
            g_score[nb] = tentative_g            # 更新代价
            heapq.heappush(open_set, (tentative_g + h(nb, goal), tie, nb))
```

关键点：`heapq` 是 Python 的最小堆，保证了每次 `heappop` 取出的都是 f 值最小的节点。

#### tie-breaker

```python
tie = 0
...
tie += 1
heapq.heappush(open_set, (f_val, tie, cell))
```

当两个节点的 f 值相同时，`tie` 作为第二排序键，保证先进先出。不加 tie-breaker 会导致两个 cell 比较时 Python 报错（因为 Cell 是 tuple，Python 会尝试比较第三个元素，可能不一致）。

#### snap_goal 和 snap_start

```python
def astar(graph, start, goal, snap_goal=True, snap_start=False):
```

默认 `snap_goal=True`（目标点自动吸附）、`snap_start=False`（起点不吸附——起点就是机器人当前位置，如果机器人站在障碍物上应该直接报错）。

#### 路径还原

```python
def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
```

`came_from` 是一棵"指针树"，每个节点指向它的父节点。从目标一直追溯到起点，然后反转，得到从起点到目标的路径。

```
came_from = {
    (3,3): (2,3),
    (2,3): (1,3),
    (1,3): (0,3),
    (0,3): (0,2),
    (0,2): (0,1),
    (0,1): (0,0),   ← 起点
}
追溯: (3,3) → (2,3) → (1,3) → (0,3) → (0,2) → (0,1) → (0,0)
反转: [(0,0), (0,1), (0,2), (0,3), (1,3), (2,3), (3,3)]
```

#### 路径转换

```python
def cell_path_to_world_path(graph, cell_path, step=1):
```

cell path 每个格子约 0.05m。如果 15m 的路径有 300 个 waypoint，每个发给 Nav2 太浪费。`step=10` 就是每隔 10 个 cell 取一个 waypoint，300 个变 30 个。

---

## 5. offline_demo.py — 离线演示

这个脚本是一个**自包含的验证工具**，不依赖 ROS 运行：

```bash
python3 -m inspection_core.offline_demo \
  --start 5.8 -3.85 --goal -4.7 1.6 --inflation 0.3
```

### 做的事情

1. 加载真实 SLAM 地图
2. 设定起点（充电区）和目标点（Meeting A）
3. 跑 A*
4. 如果 matplotlib 可用，画图可视化

### 命令行参数

```python
--map        地图 YAML 路径（默认自动查找）
--inflation  膨胀半径（米），默认 0.3
--start      起点世界坐标 x y
--goal       目标点世界坐标 x y
```

### 可视化

用 `matplotlib.imshow` 显示灰度地图，蓝色线叠画 A* 规划路径，绿色圆是起点，红色星是目标。

---

## 6. test_astar.py — 单元测试

### 为什么需要测试

你改了一行代码 → 怎么知道没把别的地方搞坏？跑一次 `pytest`，17 个测试全绿就放心了。

### 测试覆盖了什么

| 测试类 | 测试数 | 覆盖内容 |
|--------|--------|---------|
| TestGridGraph | 6 | 地图加载、坐标转换、邻居查询、墙壁跳过、膨胀 |
| TestAstar | 8 | 直线路径、死路检测、起点=终点、起点/目标在墙上、越界、最优性、绕墙 |
| TestPathHelpers | 3 | cell path 长度、世界坐标长度、step 抽稀 |

### 测试技巧：合成小地图

测试里不加载真实的 280×200 地图（太慢），而是用代码临时生成 10×10 的小地图：

```python
def _open_10x10(tmpdir):
    pixels = [[PIX_FREE] * 10 for _ in range(10)]   # 10×10 全白 = 全空地
    return _make_map(tmpdir, pixels)
```

这样每个测试在 0.01 秒内完成。

### 一个测试例子

```python
def test_straight_line(self):
    g = GridGraph(_open_10x10(td))   # 全空地 10×10
    path = astar(g, (0, 0), (9, 0)) # 从最左到最右
    assert path is not None           # 应该找到路径
    assert path[0] == (0, 0)          # 起点正确
    assert path[-1] == (9, 0)         # 终点正确
    assert len(path) == 10            # 正好 10 个格子（0 到 9）
```

### 运行测试

```bash
# 全部测试
python3 -m pytest src/inspection_core/test/test_astar.py -v

# 只跑一个测试
python3 -m pytest src/inspection_core/test/test_astar.py::TestAstar::test_straight_line -v
```

---

## 7. 面试如何讲

### 30 秒版本

> "我实现了一个基于 A* 算法的室内机器人路径规划模块。它读取 SLAM 建立的栅格地图，支持障碍物膨胀保证安全距离，用 Manhattan 距离做启发函数在 4 邻接栅格上搜索最短路径。模块是纯 Python 的，不依赖 ROS，有 17 个单元测试，可以离线验证。"

### 2 分钟版本（面试官追问时展开）

**为什么要自己写，不直接用 Nav2？**

> "Nav2 解决的是单机器人从 A 到 B 的导航问题。但我的项目最终要做的是**多机器人任务调度**——多台机器人在同一张地图上执行巡检任务，需要在上层做无冲突的时空路径规划。Nav2 的 planner 不提供 reservation table、不检测机器人之间的时空冲突。所以我自己在栅格层实现规划，把结果以 waypoint 的形式下发给 Nav2 执行。Nav2 负责底层控制，我的代码负责上层规划，各司其职。"

**A\* 为什么选 4 邻接而不是 8 邻接？**

> "两方面考虑。第一，后续做多机器人冲突检测时，4 邻接的边冲突更容易建模——A 从格子 a 移到 b，在同一时刻 B 从 b 移到 a 就是冲突。8 邻接的对角线移动会让冲突检测变得复杂。第二，Manhattan 距离是 4 邻接的可采纳启发函数，保证 A* 找到的是最短路径。"

**inflation 的作用是什么？**

> "机器人有体积，不能当质点。inflation 把障碍物向外膨胀约半个车身的距离，规划时把机器人当质点处理，路径自动保持安全距离。我用 Chebyshev 距离做膨胀，取 inflation_radius_m / resolution 个 cell 的范围。"

**snap_to_free 解决什么问题？**

> "真实 SLAM 地图不可避免地有噪声——墙壁边缘会有一些零散的 occupied cell。如果用户给的目标坐标恰好落在这些噪点上，A* 会直接失败。snap_to_free 用 BFS 从目标点向外搜索最近的 free cell，把目标吸附过去。这个设计让系统在实际使用中更鲁棒。"

**测试怎么做的？**

> "17 个 pytest 单元测试，覆盖地图加载、坐标变换、邻居查询、膨胀、直线路径、绕墙、死路检测、越界处理。每个测试用临时生成的 10×10 合成地图，不依赖真实地图，保证测试速度快、可复现。修改任何代码后跑一次 pytest 就能确认没有引入回归。"

### 关键数字（面试官喜欢听数字）

| 指标 | 数值 |
|------|------|
| 地图尺寸 | 280×200 = 56,000 cells |
| 分辨率 | 0.05 m/cell |
| 膨胀半径 | 0.3 m（约半个车宽） |
| 实测路径长度 | 充电区→前台 14.6m, 充电区→Meeting A 15.6m |
| 单元测试 | 17 个，覆盖 6 个模块 |
| A* 搜索速度 | < 0.1 秒（56,000 cell 地图） |

---

## 附录：核心代码行数

| 文件 | 行数 | 职责 |
|------|------|------|
| `grid_graph.py` | 205 | 地图加载、坐标变换、膨胀、吸附 |
| `astar.py` | 160 | A* 搜索、路径还原、waypoint 生成 |
| `offline_demo.py` | 130 | 命令行入口、matplotlib 可视化 |
| `test_astar.py` | 240 | 17 个单元测试 |

总共约 735 行 Python 代码。精而不多，每一行都有明确的用途。
