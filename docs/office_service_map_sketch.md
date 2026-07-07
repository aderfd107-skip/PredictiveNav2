# Office Service MVP Map Sketch

第一阶段 Gazebo 地图建议命名为：

```text
office_service_mvp.sdf
```

建议尺寸：

```text
14m x 10m
x: -7.0 ~ 7.0
y: -5.0 ~ 5.0
```

机器人尺寸参考：

```text
robot_1 / robot_2:
  length: about 0.70m
  width: about 0.46m
```

通道建议（2026-07-07 修改后）：

```text
main corridor: 1.8m
door opening: 1.5m - 1.7m
narrow passage (x): 1.25m
narrow passage (y): 1.25m
min robot clearance: robot is 0.46m wide, so min passage >= 1.0m
```

## 1. Top-Down Sketch

```text
                         y = +5.0
        x=-7.0                                      x=+7.0
          +------------------------------------------------+
          | Meeting A        | Equipment    | Meeting B     |
          |                  | Room         |               |
          |   IA             |   IE         |   IB          |
          |                  |              |               |
          |------- door -----+---- door ----+----- door -----|
          |                                                |
          |                                                |
          |============== MAIN CORRIDOR ===================|
          |                                                |
          |  P1      T-junction / conflict zone       P2   |
          |                                                |
          |---- door --------+--------------+---- narrow ---|
          | Front Desk       | Office Area  |  Passage      |
          | / Service Counter|              |               |
          |   PF             |   IO         |      NP       |
          |                  |              |               |
          |------------------+----- door ----+--------------|
          | Storage Room     | Printer Area | Charging Zone |
          |   PS             |   IP         |  C1   C2      |
          +------------------------------------------------+
                         y = -5.0
```

Legend:

```text
PF: pickup point at front desk
PS: pickup point at storage room
IA: inspection point in Meeting A
IB: inspection point in Meeting B
IE: inspection point in Equipment Room
IO: office area service point
IP: printer area inspection point
NP: narrow passage test area
C1/C2: charging / standby positions
P1/P2: robot initial positions or waiting points
```

## 2. Suggested Room Layout

外墙：

```text
world boundary:
  west wall:  x = -7.0
  east wall:  x = +7.0
  south wall: y = -5.0
  north wall: y = +5.0
```

上半区：

```text
Meeting A:
  x: -6.8 ~ -2.4
  y:  2.0 ~  4.8

Equipment Room:
  x: -2.4 ~  1.6
  y:  2.0 ~  4.8

Meeting B:
  x:  1.6 ~  6.8
  y:  2.0 ~  4.8
```

中间：

```text
Main Corridor:
  x: -6.8 ~ 6.8
  y: -0.9 ~ 0.9
  width: 1.8m
```

下半区：

```text
Front Desk:
  x: -6.8 ~ -3.0
  y: -3.0 ~ -1.0

Office Area:
  x: -3.0 ~  1.5
  y: -3.0 ~ -1.0

Narrow Passage:
  x:  1.5 ~  3.0
  y: -3.0 ~ -1.0
  passage width target: about 1.0m

Storage Room:
  x: -6.8 ~ -3.0
  y: -4.8 ~ -3.0

Printer Area:
  x: -3.0 ~  1.5
  y: -4.8 ~ -3.0

Charging Zone:
  x:  3.0 ~  6.8
  y: -4.8 ~ -3.0
```

## 3. Suggested Task Points

```text
robot_1_start: [-5.8,  0.0, 0.0]
robot_2_start: [ 5.8,  0.0, 3.14]

pickup_front_desk: [-5.5, -1.8, 0.0]
pickup_storage:    [-5.5, -3.8, 0.0]

dropoff_meeting_a: [-4.6,  3.2, 0.0]
dropoff_meeting_b: [ 4.2,  3.2, 3.14]
dropoff_office:    [-0.8, -1.8, 0.0]

inspect_equipment: [-0.4,  3.2, 0.0]
inspect_printer:   [-0.8, -3.8, 0.0]

charging_1: [4.4, -3.9, 1.57]
charging_2: [5.6, -3.9, 1.57]
```

## 4. Why This Map Is Better Than The Previous Project

旧项目最大地图约为：

```text
10m x 8m = 80 square meters
```

本地图建议为：

```text
14m x 10m = 140 square meters
```

提升点：

```text
- area is about 1.75x larger
- has clear office service semantics
- supports pickup/dropoff tasks
- supports inspection tasks
- has a main corridor for multi-robot encounters
- has a narrow passage for conflict-free planning tests
- has charging/standby positions for future scheduling
```

## 5. First Gazebo Version Boundary

第一版 Gazebo world 先做：

```text
- walls
- room partitions
- simple doors/openings
- service counter
- storage shelf
- printer block
- charging station blocks
- visual task markers
```

第一版 Gazebo world 暂不做：

```text
- moving people
- elevators
- doors that open/close
- detailed furniture
- real item loading animation
- multi-floor structure
```
