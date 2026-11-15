# PND Adam Pro 运行与开发手册

本文对应 `pnd-adam-ros2-mujoco` 的默认 **MuJoCo → DDS → ROS2 Humble → MoveIt2 / RViz** 链路。除特别注明外，命令均在仓库根目录执行；拆分启动时，每个终端都先进入同一仓库根目录。

[项目首页](README.md) · [系统架构](ARCHITECTURE.md) · [状态与已知问题](PROJECT_STATUS.md) · [关节限位审计](JOINT_LIMIT_AUDIT.md)

## 1. 环境与首次安装

目标环境为 Ubuntu 22.04、ROS2 Humble、Python 3.10，以及支持 OpenGL 的桌面环境。先安装 ROS2 Humble，确认 `/opt/ros/humble/setup.bash` 存在；本仓库的安装脚本不会安装 ROS2 本体。

```bash
git clone https://github.com/hugh82617/pnd-adam-ros2-mujoco.git
cd pnd-adam-ros2-mujoco
bash scripts/setup_environment.sh
```

脚本会通过 `sudo apt-get` 安装系统依赖，通过当前用户的 `pip` 安装 Python 依赖及仓库内 SDK，并编译三个当前 ROS2 包。需要可访问的 apt / Python 软件源。具体实现见 [`scripts/setup_environment.sh`](scripts/setup_environment.sh)。

<a id="manual-install"></a>

### 手动安装依赖

以下内容与安装脚本一致；已成功运行安装脚本时无需重复执行。

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-yaml \
  python3-colcon-common-extensions \
  libyaml-cpp-dev \
  libspdlog-dev \
  libboost-all-dev \
  libglfw3-dev \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-planners-chomp \
  ros-humble-pilz-industrial-motion-planner \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-sensor-msgs

python3 -m pip install --user -r requirements.txt
python3 -m pip install --user -e ./pnd_sdk_python
```

[`requirements.txt`](requirements.txt) 固定 `mujoco==3.2.0`、`cyclonedds==0.10.2`，并安装 NumPy、OpenCV、Pygame、PyYAML。应使用本仓库中的 SDK 快照，以保持 IDL 与适配器一致。

如果 Cyclone DDS 安装报 `Could not locate cyclonedds`，参考 [SDK 安装说明](pnd_sdk_python/README.md) 中的源代码安装方法，配置 `CYCLONEDDS_HOME` 后再安装 SDK；不要直接替换 SDK 或 IDL 版本来绕过该错误。

### 编译工作区

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description pnd_ros2_control pnd_adam_moveit_config
source install/setup.bash
```

`legacy/` 通过 `COLCON_IGNORE` 排除在当前构建之外。修改源码或配置后重新编译，随后重启相关节点；在新终端中也需要重新 `source`。

## 2. 一键启动、日志与停止

```bash
bash start_pnd_adam.sh
```

脚本按所在位置定位仓库，启动 MuJoCo、控制桥、DDS 关节状态发布节点、机器人 TF、MoveIt `move_group` 和 RViz。不要求仓库位于固定的 home 子目录。

启动终端会持续显示日志；默认文件为：

```text
/tmp/pnd_mujoco.log
/tmp/pnd_ros2_control.log
/tmp/pnd_moveit.log
```

需要单独查看时，在另一个终端运行：

```bash
tail -f /tmp/pnd_mujoco.log /tmp/pnd_ros2_control.log /tmp/pnd_moveit.log
```

也可以在启动时指定日志目录，脚本会创建该目录：

```bash
PND_LOG_DIR="$PWD/run-logs" bash start_pnd_adam.sh
```

在**启动脚本所在终端**按 `Ctrl+C`，脚本会停止其创建的进程组。仅在另开的日志查看终端按 `Ctrl+C`，只会结束那次日志查看。启动脚本输出“started”表示已发起进程，不代替下文的就绪检查；自定义日志文件默认会在再次启动时覆盖。

## 3. 等价的三终端启动

以下方法用于分别观察日志或调整参数。先完成安装和编译，停掉已有的一键启动实例；每个终端都从仓库根目录开始。

### 终端 1：MuJoCo

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cd pnd_mujoco/simulate_python
python3 pnd_mujoco.py
```

MuJoCo 入口依赖当前目录解析场景文件，并且即使使用 DDS 模式也会导入 `rclpy`，所以此终端同样需要 ROS2 环境。

### 终端 2：控制与状态反馈

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pnd_ros2_control pnd_control.launch.py
```

此命令启动 `pnd_ros2_control_bridge`、`pnd_joint_state_publisher`、`robot_state_publisher` 和静态 `world → base_link` TF。正式 `/joint_states` 由 DDS 状态转换得到。

### 终端 3：MoveIt2 与 RViz

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

关闭重复的状态发布器，因为终端 2 已提供 TF。不启动 RViz 时再添加 `start_rviz:=false`。当前默认 `use_sim_time:=false`，不要在没有 `/clock` 的链路中单独改成 `true`。

三终端模式直接在各终端显示日志，不自动创建一键脚本的三份日志文件。结束实验时分别在终端 3、2、1 按 `Ctrl+C`。

## 4. 就绪检查与 RViz 操作

在第四个终端进入仓库根目录，加载环境后检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
ros2 action list -t
ros2 run tf2_ros tf2_echo world left_wrist_roll_link
```

`tf2_echo` 持续运行，查看后按 `Ctrl+C` 退出。默认关节状态应包含 **43 项：31 个身体关节 + 12 个手部控制关节**；`/joint_states` 应只有当前链路的正式发布源。TF 由机器人描述与关节状态生成，`world → base_link` 是固定变换，不是自由基座反馈。

MoveIt 使用以下 5 个轨迹 Action：

```text
/body_controller/follow_joint_trajectory
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
/left_hand_controller/follow_joint_trajectory
/right_hand_controller/follow_joint_trajectory
```

桥接节点还提供 `/body_joint_trajectory_controller/follow_joint_trajectory` 兼容别名，所以 `ros2 action list` 可能看到 6 个接口。接口存在只证明服务已注册，不能证明轨迹已成功执行。

原项目记录的正常启动日志包括：

```text
LowState received; control bridge is active
Idle hold active: kp=5.0, kd=1.0
You can start planning now!
```

在 RViz 的 MotionPlanning 面板中选择 `right_arm` 或 `left_arm`，调整目标并先执行 `Plan`，确认轨迹后执行 `Plan & Execute`，同时观察 MuJoCo 的实际关节运动及 `/joint_states` 反馈。原记录已验证身体与双臂执行；双手独立轨迹、头部 IK 等状态见 [PROJECT_STATUS.md](PROJECT_STATUS.md)。

## 5. 修改场景与关键参数

### MuJoCo 场景

编辑 [`pnd_mujoco/simulate_python/config.py`](pnd_mujoco/simulate_python/config.py)：

```python
ROBOT = "adam_pro"
ROBOT_SCENE = "adam_proximity_scene.xml"
SDK_TYPE = "DDS"
DOMAIN_ID = 1
INTERFACE = "lo"
ENABLE_ROS2_OUTPUT = False
```

场景在仿真目录中相对加载，可将 `ROBOT_SCENE` 改为仓库已有的 `adam_pro_pulse_scene.xml`。场景名称不代表已经实现对应的感知或任务算法；切换后需重新启动仿真和控制链路。

保持 `ENABLE_ROS2_OUTPUT=False`，让独立 ROS2 节点负责正式状态输出。默认仿真 DDS 使用 domain `1` 和本机回环接口 `lo`；若修改，仿真端与 ROS2 控制层的 `domain_id`、`interface` 必须同时匹配。这些是 PND DDS 连接参数，不应直接等同于 ROS2 的 `ROS_DOMAIN_ID` 环境变量。

### 控制桥与状态频率

需要临时调整时，使用三终端启动方式，在终端 2 替换启动命令：

```bash
ros2 launch pnd_ros2_control pnd_control.launch.py \
  domain_id:=1 interface:=lo \
  control_rate:=500.0 state_publish_rate:=100.0 \
  publish_hand_joints:=true idle_kp:=5.0 idle_kd:=1.0
```

这些是配置目标频率，实际时序需通过运行测量确认。`idle_kp`、`idle_kd` 是空闲保位增益；轨迹执行所用的关节默认增益和顺序由关节注册表定义。

| 需要修改的内容 | 文件 |
|---|---|
| 电机顺序、SDK / ROS 映射、关节默认 Kp / Kd | [`pnd_joint_registry.yaml`](src/pnd_ros2_control/config/pnd_joint_registry.yaml) |
| MuJoCo 电机与手部映射 | [`motor_mapping.yaml`](pnd_mujoco/simulate_python/motor_mapping.yaml)、[`hand_mapping.yaml`](pnd_mujoco/simulate_python/hand_mapping.yaml) |
| MoveIt 关节限位 | [`joint_limits.yaml`](src/pnd_adam_moveit_config/config/joint_limits.yaml) |
| 双臂 KDL IK | [`kinematics.yaml`](src/pnd_adam_moveit_config/config/kinematics.yaml) |
| 控制器 Action 与关节组 | [`moveit_controllers.yaml`](src/pnd_adam_moveit_config/config/moveit_controllers.yaml) |
| URDF、碰撞几何和 TF | [`pnd_adam_description`](src/pnd_adam_description/README.md) |

修改关节映射、URDF、SRDF、MoveIt 配置后重新编译并重启整栈。历史模型生成脚本仍带旧目录默认值；运行前需审查其输入、输出与手部处理，本文不提供直接覆盖现有模型的默认生成命令。

## 6. 运行现有测试与限位审计

### ROS2 控制层测试

在已安装依赖、编译完成并加载 ROS2 工作区环境的终端中运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/pnd_ros2_control/test/test_command_interface.py
python3 src/pnd_ros2_control/test/test_joint_state_publisher.py
```

这些脚本直接调用仓库已有测试函数，无需额外安装 pytest。它们检查 31 / 12 关节注册、命令映射、空闲保持和 43 项 JointState 构造，不需要启动仿真，也不验证 MoveIt 的完整运行闭环。

### 仿真适配器测试

以下两个已有测试脚本在仿真目录内运行；使用子 shell，执行结束后当前终端仍位于仓库根目录：

```bash
(cd pnd_mujoco/simulate_python && python3 test_phase2_adapter.py)
(cd pnd_mujoco/simulate_python && python3 test_phase3_ros2_adapter.py)
```

前者加载 MuJoCo 模型，检查电机映射、手部转换和适配逻辑；后者检查旧 ROS2 适配器的消息构造。后者通过不表示旧适配器已成为默认状态来源。目录中的 DDS 发布、订阅及演示脚本会连接通信或发送命令，不应当作纯单元测试批量启动。

### 重新生成关节限位审计

```bash
python3 scripts/generate_joint_limit_audit.py
git diff -- JOINT_LIMIT_AUDIT.md
```

该脚本读取 MJCF、当前 URDF 与 MoveIt 限位，**重写**根目录 `JOINT_LIMIT_AUDIT.md`。审计采用 31 个身体关节初始值为零的假设；目前膝关节下限为 `0.052`，因此初始零位会被报告为冲突。它不修改限位或修复姿态，也不审计手部与动态跟踪效果。

## 7. 常见问题

| 现象 | 优先检查 |
|---|---|
| `ROS 2 Humble was not found` | 先安装目标 ROS2 发行版，确认 `/opt/ros/humble/setup.bash` |
| MuJoCo 报 `No module named rclpy` | 仿真终端也要 source ROS2；使用与 Humble 对应的系统 Python |
| MuJoCo 找不到 XML / 网格 | 从 `pnd_mujoco/simulate_python` 启动入口，核对场景与仓库资产路径 |
| 没有 `LowState received` 或 watchdog 超时 | 确认 MuJoCo 正常运行，核对 domain / interface，并查看仿真和桥接日志 |
| RViz 模型跳动或关节突然归零 | 检查 `/joint_states` 发布者数量，停止旧 fake publisher 或重复实例 |
| MoveIt 没有当前状态或执行无响应 | 核对 43 项状态、Action 接口、控制桥日志及是否重复发布 TF |
| 头部或手部没有末端交互标记 | 当前 KDL IK 只配置双臂；手部更适合独立关节轨迹验证 |
| 膝关节起始状态被调整 | 对照[限位审计](JOINT_LIMIT_AUDIT.md)，核对初始姿态与最小值 |

[`legacy/run_publisher.sh`](legacy/run_publisher.sh) 属于旧版假状态示例，不要与默认链路同时运行。完整能力边界和后续计划见[项目状态](PROJECT_STATUS.md)，第三方组件来源见[许可记录](docs/third-party-notices.md)。
