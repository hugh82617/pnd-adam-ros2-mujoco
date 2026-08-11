# PND Adam ROS2 MoveIt Simulation

本项目整合了 MuJoCo 仿真、PND DDS、ROS2 Humble、MoveIt2 和 RViz，用于复现 Adam Pro 机器人仿真控制链路。

## 文档

- [PROJECT_STATUS.md](PROJECT_STATUS.md)：当前阶段、已验证能力、已知问题
- [ARCHITECTURE.md](ARCHITECTURE.md)：完整系统架构与控制/反馈路径
- [PROJECT_COMMANDS.md](PROJECT_COMMANDS.md)：启动、编译、修改、验证命令
- [JOINT_LIMIT_AUDIT.md](JOINT_LIMIT_AUDIT.md)：关节限位与官方初始 pose 审计

## 仓库结构

```text
.
├── ARCHITECTURE.md
├── JOINT_LIMIT_AUDIT.md
├── PROJECT_COMMANDS.md
├── PROJECT_STATUS.md
├── README.md
├── legacy/                       # 旧版 demo / 废弃脚本，保留但不参与当前链路
├── pnd_mujoco/                      # MuJoCo 仿真代码与 Adam Pro 模型
│   └── simulate_python/             # 仿真入口 pnd_mujoco.py
├── pnd_sdk_python/                  # PND Python DDS SDK
├── src/pnd_adam_description/        # URDF、mesh、joint_limits、生成脚本
├── src/pnd_adam_moveit_config/      # MoveIt2 SRDF、IK、控制器、RViz 配置
├── src/pnd_ros2_control/            # ROS2 控制桥、关节状态发布、Action Server
├── src/pnd_joint_state_demo/        # 旧版关节状态发布示例
├── start_pnd_adam.sh                # 一键启动脚本
└── PROJECT_COMMANDS.md              # 命令速查
```

## 在其他电脑上复现

### 1. 系统要求

- Ubuntu 22.04
- ROS2 Humble 已安装
- Python 3.10
- 支持 OpenGL 的桌面环境，用于 MuJoCo 和 RViz

### 2. 克隆仓库

```bash
git clone https://github.com/hugh82617/0811.git
cd 0811
```

### 3. 自动安装依赖并编译

```bash
bash scripts/setup_environment.sh
```

该脚本会安装：

- MoveIt2 相关 ROS 包
- MuJoCo Python 依赖
- PND Python DDS SDK
- 编译当前 ROS2 workspace

### 4. 手动安装依赖

如果不想使用自动脚本，可以手动执行：

```bash
sudo apt update
sudo apt install -y \
  python3-pip python3-yaml \
  libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev \
  ros-humble-moveit ros-humble-moveit-setup-assistant \
  ros-humble-moveit-kinematics ros-humble-moveit-planners-ompl \
  ros-humble-moveit-planners-chomp ros-humble-pilz-industrial-motion-planner \
  ros-humble-xacro ros-humble-robot-state-publisher

python3 -m pip install --user -r requirements.txt
python3 -m pip install --user -e ./pnd_sdk_python
```

### 5. 编译 ROS2 workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description pnd_ros2_control pnd_adam_moveit_config
source install/setup.bash
```

### 6. 启动完整仿真

```bash
bash start_pnd_adam.sh
```

启动内容：

- MuJoCo：`pnd_mujoco/simulate_python/pnd_mujoco.py`
- ROS2 控制桥：`pnd_ros2_control_bridge`
- 关节状态发布：`pnd_joint_state_publisher`
- MoveIt2：`move_group`
- RViz：MoveIt MotionPlanning

### 7. 验证是否正常

```bash
tail -f /tmp/pnd_mujoco.log /tmp/pnd_ros2_control.log /tmp/pnd_moveit.log
```

正常日志应包含：

```text
LowState received; control bridge is active
Idle hold active: kp=5.0, kd=1.0
You can start planning now!
```

也可以在 RViz 中：

1. 选择 Planning Group：`right_arm`
2. 点击 Plan
3. 点击 Plan & Execute

## 说明

- `pnd_mujoco` 中的模型路径已改为仓库内相对路径，clone 后不需要修改 `/home/zch` 绝对路径。
- 机器人 mesh 统一放在 `src/pnd_adam_description/meshes`。
- `legacy/` 保存旧版 demo、旧构建脚本和旧版 fake publisher，不参与当前控制链路。
- 不要运行 `legacy/run_publisher.sh`，它会把假关节状态发布到 `/joint_states`，覆盖真实状态。
