# Path Tracking Simulator - TurtleBot3 版本

基於 [原作者的 YouTube 教學](https://www.youtube.com/watch?v=uls-WmxRiTw) 改寫，使用 Gazebo 11 和 TurtleBot3 機器人的路徑追蹤模擬器。

## 專案簡介

這是一個機器人路徑追蹤模擬系統，提供多種路徑追蹤控制算法，並配備了圖形化界面來繪製和測試機器人的路徑追蹤能力。

### 主要特性

- **多種控制算法**：
  - Pure Pursuit Controller（純追蹤控制器）- 適用於 TurtleBot3
  - Model Predictive Control (MPC)（模型預測控制）- 適用於 Ackermann 轉向機器人
  - Pure Pursuit for Ackermann（Ackermann 轉向純追蹤）

- **圖形化路徑規劃**：
  - PyQt5 界面，可用滑鼠繪製自定義路徑
  - 實時機器人位置顯示
  - 路徑可視化

- **模擬環境**：
  - Gazebo 11 物理模擬器
  - TurtleBot3 Burger 機器人模型
  - 自定義模擬場景

## 系統需求

- Ubuntu 20.04 / 22.04
- ROS 2 (Foxy / Humble)
- Gazebo 11
- Python 3.8+

### 依賴套件

```bash
# ROS 2 基礎套件
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install ros-${ROS_DISTRO}-robot-state-publisher
sudo apt install ros-${ROS_DISTRO}-xacro

# TurtleBot3 相關套件
sudo apt install ros-${ROS_DISTRO}-turtlebot3
sudo apt install ros-${ROS_DISTRO}-turtlebot3-simulations

# Python 套件
pip3 install numpy
pip3 install opencv-python
pip3 install PyQt5
pip3 install cvxpy
```

## 專案結構

```
path_tracking_tb3/
├── UI/                                    # PyQt5 圖形界面
│   ├── path_tracking_simulator.py        # 主界面程式
│   └── path_tracking_simulator_window.py # UI 設計檔
├── src/
│   ├── robot_control/                    # 控制算法
│   │   └── scripts/
│   │       ├── pure_pursuit_tb3.py       # TurtleBot3 純追蹤控制
│   │       ├── pure_pursuit_ackermann.py # Ackermann 純追蹤控制
│   │       ├── model_predictive_control.py # MPC 控制
│   │       └── test_path_publisher.py    # 路徑發布測試
│   └── robot_simulation/                 # Gazebo 模擬相關
│       ├── robot_description/            # 機器人描述檔案
│       │   ├── turtlebot3/              # TurtleBot3 URDF
│       │   └── robot/                   # 自定義機器人模型
│       └── robot_gazebo/                # Gazebo 啟動檔和世界檔
│           ├── launch/
│           │   ├── main_tb3.launch.py   # TurtleBot3 啟動檔
│           │   ├── main_ackermann.launch.py # Ackermann 啟動檔
│           │   └── main_original.launch.py  # 原始機器人啟動檔
│           └── worlds/                  # Gazebo 世界檔
```

## 安裝與編譯

1. **Clone 專案**
```bash
cd ~/
git clone <your-repo-url> path_tracking_tb3
cd path_tracking_tb3
```

2. **編譯工作空間**
```bash
colcon build --symlink-install
source install/setup.bash
```

3. **設置環境變數**（可選，加入 ~/.bashrc）
```bash
echo "source ~/path_tracking_tb3/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 使用方法

### 1. 啟動 Gazebo 模擬環境

**使用 TurtleBot3**（差速驅動）：
```bash
ros2 launch robot_gazebo main_tb3.launch.py
```

**使用 Ackermann 轉向機器人**：
```bash
ros2 launch robot_gazebo main_ackermann.launch.py
```

### 2. 啟動路徑規劃 GUI

開啟新終端：
```bash
cd ~/path_tracking_tb3/UI
python3 path_tracking_simulator.py
```

在 GUI 界面中：
1. 用滑鼠拖曳繪製路徑（藍色線條）
2. 點擊「Publish Path」按鈕發布路徑到機器人

### 3. 啟動控制器

開啟新終端：

**TurtleBot3 Pure Pursuit 控制器**：
```bash
cd ~/path_tracking_tb3/src/robot_control/scripts
python3 pure_pursuit_tb3.py
```

**Ackermann Pure Pursuit 控制器**：
```bash
cd ~/path_tracking_tb3/src/robot_control/scripts
python3 pure_pursuit_ackermann.py
```

**MPC 控制器**：
```bash
cd ~/path_tracking_tb3/src/robot_control/scripts
python3 model_predictive_control.py
```

## 控制算法說明

### Pure Pursuit（純追蹤控制）

Pure Pursuit 是一種幾何路徑追蹤算法：
- **Look-ahead distance（前視距離）**：0.5 m（TurtleBot3）
- **目標速度**：0.3 m/s
- **最大角速度**：1.5 rad/s
- 適用於差速驅動機器人（如 TurtleBot3）

工作原理：
1. 在路徑上尋找前視距離處的目標點
2. 計算機器人到目標點的角度差
3. 根據角度差計算所需的角速度

### Model Predictive Control（模型預測控制）

MPC 是一種基於優化的控制方法：
- **預測時域**：1 步
- **控制變量**：加速度和轉向角
- **求解器**：CVXPY (CLARABEL)
- 適用於 Ackermann 轉向機器人

優勢：
- 可處理約束條件（速度、轉向角限制）
- 預測未來軌跡進行優化
- 更平滑的控制輸出

## ROS 2 Topics

### 訂閱的 Topics

| Topic | 訊息類型 | 說明 |
|-------|---------|------|
| `/path` | `std_msgs/Float64MultiArray` | 從 GUI 接收規劃的路徑 |
| `/odom` | `nav_msgs/Odometry` | 機器人的里程計資訊（TurtleBot3）|
| `/model/fws_robot/pose` | `tf2_msgs/TFMessage` | 機器人位姿（Ackermann）|

### 發布的 Topics

| Topic | 訊息類型 | 說明 |
|-------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（TurtleBot3）|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 轉向位置指令（Ackermann）|
| `/forward_velocity_controller/commands` | `std_msgs/Float64MultiArray` | 輪速指令（Ackermann）|

## 參數調整

### Pure Pursuit 參數（pure_pursuit_tb3.py）

```python
k = 0.1          # look forward gain（前視增益）
Lfc = 0.5        # look-ahead distance（前視距離，米）
Kp = 1.0         # speed proportional gain（速度比例增益）
target_speed = 0.3  # 目標速度（m/s）
max_angular_vel = 1.5  # 最大角速度（rad/s）
```

### MPC 參數（model_predictive_control.py）

```python
T = 1            # 預測時域長度
R = np.diag([0.01, 0.01])     # 輸入成本矩陣
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # 狀態成本矩陣
TARGET_SPEED = 8.0 / 3.6  # 目標速度（m/s）
MAX_STEER = np.deg2rad(60.0)  # 最大轉向角（弧度）
```

## 故障排除

### 問題：Gazebo 無法啟動或找不到模型

**解決方法**：
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/path_tracking_tb3/install/robot_gazebo/share/robot_gazebo/worlds
```

### 問題：機器人不移動

**檢查項目**：
1. 確認 GUI 已發布路徑（終端應顯示 "Received path with X points"）
2. 確認控制器腳本正在運行
3. 檢查 ROS 2 topics：`ros2 topic list`
4. 檢查訊息流：`ros2 topic echo /cmd_vel`

### 問題：路徑追蹤不準確

**調整建議**：
- 降低目標速度
- 增加前視距離（Lfc）
- 調整 Kp 增益值
- 確保路徑點間距合適（約 0.2 m）

## 致謝

- 原作者：[YouTube 教學影片](https://www.youtube.com/watch?v=uls-WmxRiTw)
- 改寫為 Gazebo 11 + TurtleBot3 版本

## 授權

本專案基於原作者的工作改編，使用時請遵守相關開源協議。

## 相關資源

- [ROS 2 官方文檔](https://docs.ros.org/en/humble/)
- [TurtleBot3 文檔](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo 教學](http://gazebosim.org/tutorials)
- [Pure Pursuit 算法解說](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- [MPC 控制介紹](https://en.wikipedia.org/wiki/Model_predictive_control)
