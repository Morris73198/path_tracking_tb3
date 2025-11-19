# 路径误差监测功能使用说明

## 概述

本项目已添加实时 x-y 路径误差监测和可视化功能。两个路径跟踪控制器（MPC 和 Pure Pursuit）现在都会发布路径误差数据到 `/path_error` topic。

## 误差数据格式

`/path_error` topic 发布 `Float64MultiArray` 消息，包含以下 4 个数值：

| 索引 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 0 | x_error | m | X 方向位置误差（目标点 X - 当前 X） |
| 1 | y_error | m | Y 方向位置误差（目标点 Y - 当前 Y） |
| 2 | cross_track_error | m | 横向误差（垂直于路径的距离） |
| 3 | heading_error | rad | 航向误差（目标航向 - 当前航向） |

## 使用方法

### 方法 1：使用提供的可视化脚本（推荐）

1. **启动仿真环境和路径跟踪控制器**

```bash
# 终端 1: 启动 Gazebo 仿真
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2: 启动路径规划 GUI
ros2 run robot_control path_tracking_simulator.py

# 终端 3: 启动 MPC 控制器（或 Pure Pursuit）
ros2 run robot_control mpc_tb3.py
# 或
ros2 run robot_control pure_pursuit_tb3.py
```

2. **启动实时误差监测可视化**

```bash
# 终端 4: 启动误差绘图工具
ros2 run robot_control plot_path_error.py
```

或者直接运行：
```bash
python3 src/robot_control/scripts/plot_path_error.py
```

3. **在 GUI 中绘制路径并发布**

可视化窗口将显示：
- **上图**：X 误差（红）、Y 误差（绿）、横向误差（蓝）
- **下图**：航向误差（品红）

### 方法 2：使用 ROS 命令行工具

**查看实时误差数据**
```bash
ros2 topic echo /path_error
```

**使用 rqt_plot 绘制波形图**
```bash
# 安装 rqt_plot（如果还没有安装）
sudo apt install ros-humble-rqt-plot

# 绘制横向误差
rqt_plot /path_error/data[2]

# 绘制所有误差
rqt_plot /path_error/data[0]:data[1]:data[2]:data[3]
```

**使用 PlotJuggler（推荐用于高级分析）**
```bash
# 安装 PlotJuggler
sudo apt install ros-humble-plotjuggler-ros

# 启动 PlotJuggler
ros2 run plotjuggler plotjuggler
```

在 PlotJuggler 中：
1. Streaming → Start: ROS2 Topic Subscriber
2. 选择 `/path_error`
3. 拖拽数据到图表中

### 方法 3：录制数据供后续分析

```bash
# 录制误差数据
ros2 bag record /path_error

# 回放数据
ros2 bag play <your_bag_file>
```

## 可视化脚本功能

- **实时波形图**：matplotlib 动画显示误差随时间变化
- **自动缩放**：坐标轴自动调整以适应数据范围
- **数据缓冲**：默认显示最近 500 个数据点（约 50 秒，MPC 控制器）
- **性能优化**：使用 deque 实现高效数据管理

## 修改的文件

1. **src/robot_control/scripts/mpc_tb3.py**
   - 添加了 `/path_error` 发布者
   - 在控制循环中计算并发布误差（行 441-454）

2. **src/robot_control/scripts/pure_pursuit_tb3.py**
   - 添加了 `/path_error` 发布者
   - 在控制循环中计算并发布误差（行 193-224）

3. **src/robot_control/scripts/plot_path_error.py** （新文件）
   - ROS 2 节点用于实时误差可视化

## 故障排除

### 没有显示数据？

检查是否有控制器在运行并且已经发布了路径：
```bash
ros2 topic list | grep path_error
ros2 topic hz /path_error
```

### matplotlib 错误？

确保已安装 matplotlib：
```bash
pip3 install matplotlib
```

### 图表更新很慢？

可以调整可视化脚本中的参数：
- `max_points=500` → 减少此值以提高性能
- `interval=100` → 增加此值以降低更新频率

## 技术细节

### 误差计算方法

**MPC 控制器：**
- 使用 `calc_nearest_index()` 计算横向误差（有符号距离）
- 基于参考轨迹的最近点计算位置误差

**Pure Pursuit 控制器：**
- 使用欧几里得距离计算最近点
- 基于路径切线方向计算航向误差

### 发布频率

- **MPC**：10 Hz（DT = 0.1s）
- **Pure Pursuit**：50 Hz（time_interval = 0.02s）

## 下一步改进建议

1. 添加误差统计（均值、标准差、最大值）
2. 实现误差阈值报警
3. 保存误差数据到 CSV 文件
4. 添加对比多个控制器性能的功能
5. 集成到 PyQt5 GUI 中

## 示例输出

```
[INFO] [path_error_plotter]: Path Error Plotter initialized
[INFO] [path_error_plotter]: Waiting for /path_error messages...
[INFO] [path_error_plotter]: Received 50 error messages. CTE: 0.0234m, Heading: 0.0512rad
[INFO] [path_error_plotter]: Received 100 error messages. CTE: 0.0187m, Heading: 0.0324rad
```

## 参考资料

- [ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Topics.html)
- [matplotlib Animation](https://matplotlib.org/stable/api/animation_api.html)
- [PlotJuggler Documentation](https://github.com/facontidavide/PlotJuggler)
