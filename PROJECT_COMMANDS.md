# PND Adam ROS2 项目命令总结

本文档整合 `start_pnd_adam.sh` 的启动流程、脚本展开说明、常用修改命令和验证命令。

## 文档索引

- [README.md](README.md)：项目说明与在其他电脑上复现
- [PROJECT_STATUS.md](PROJECT_STATUS.md)：当前状态和已知问题
- [ARCHITECTURE.md](ARCHITECTURE.md)：系统架构
- [JOINT_LIMIT_AUDIT.md](JOINT_LIMIT_AUDIT.md)：关节限位审计

`legacy/` 存放旧版官方 demo 和废弃脚本，不参与当前 MuJoCo + ROS2 + MoveIt2 控制链路。

## 1. 一键启动/停止

```bash
bash ~/pnd_adam_ros2_publish/start_pnd_adam.sh
```

这个脚本会同时启动：

- MuJoCo 仿真：`pnd_mujoco.py`
- ROS2 控制桥：`pnd_ros2_control_bridge`
- 关节状态发布：`pnd_joint_state_publisher`
- MoveIt2 + RViz

终端 1：

```bash
cd ~/pnd_adam_ros2_publish/pnd_mujoco/simulate_python
python3 pnd_mujoco.py
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
cd ~/pnd_adam_ros2_publish
source install/setup.bash
ros2 launch pnd_ros2_control pnd_control.launch.py
```

终端 3：

```bash
source /opt/ros/humble/setup.bash
cd ~/pnd_adam_ros2_publish
source install/setup.bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

停止：在启动终端按 `Ctrl+C`。

日志实时查看：

```bash
tail -f /tmp/pnd_mujoco.log /tmp/pnd_ros2_control.log /tmp/pnd_moveit.log
```

也可以自定义日志目录：

```bash
PND_LOG_DIR=~/pnd_logs bash ~/pnd_adam_ros2_publish/start_pnd_adam.sh
```

## 2. start_pnd_adam.sh 脚本原文

```bash
#!/usr/bin/env bash
set -u

ROOT="$HOME/pnd_adam_ros2_publish"
SIM_DIR="$ROOT/pnd_mujoco/simulate_python"
LOG_DIR="${PND_LOG_DIR:-/tmp}"
PIDS=()
export ROOT SIM_DIR

cleanup() {
  echo ""
  echo "Stopping PND Adam stack..."
  for pid in "${PIDS[@]}"; do
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  sleep 1
  wait 2>/dev/null || true
  echo "Stopped."
}

trap cleanup EXIT INT TERM

setsid bash -c 'cd "$SIM_DIR" && exec python3 pnd_mujoco.py' \
  > "$LOG_DIR/pnd_mujoco.log" 2>&1 &
PIDS+=($!)

setsid bash -c '
  source /opt/ros/humble/setup.bash
  cd "$ROOT"
  source install/setup.bash
  exec ros2 launch pnd_ros2_control pnd_control.launch.py
' > "$LOG_DIR/pnd_ros2_control.log" 2>&1 &
PIDS+=($!)

setsid bash -c '
  source /opt/ros/humble/setup.bash
  cd "$ROOT"
  source install/setup.bash
  exec ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
    start_robot_state_publisher:=false
' > "$LOG_DIR/pnd_moveit.log" 2>&1 &
PIDS+=($!)

sleep 2

echo "PND Adam stack started."
echo "Logs:"
echo "  $LOG_DIR/pnd_mujoco.log"
echo "  $LOG_DIR/pnd_ros2_control.log"
echo "  $LOG_DIR/pnd_moveit.log"
echo "Press Ctrl-C to stop all processes."

tail -f \
  "$LOG_DIR/pnd_mujoco.log" \
  "$LOG_DIR/pnd_ros2_control.log" \
  "$LOG_DIR/pnd_moveit.log"
```

## 3. 脚本逐段拆解

- `set -u`
  脚本中变量未定义时直接报错，避免拼错变量后继续运行。

- `ROOT`、`SIM_DIR`
  定义 ROS2 workspace 和 MuJoCo 代码目录。

- `LOG_DIR="${PND_LOG_DIR:-/tmp}"`
  默认日志写到 `/tmp`，可以通过环境变量 `PND_LOG_DIR` 修改。

- `cleanup()`
  停止所有进程。每个子进程都用 `setsid` 启动，所以能用 `kill -- "-$pid"` 按进程组整体杀掉，避免留下 MuJoCo、ROS2 或 MoveIt 孤儿进程。

- `trap cleanup EXIT INT TERM`
  按下 `Ctrl+C`、终端关闭或脚本退出时自动调用 `cleanup`。

- `setsid bash -c ... &`
  启动独立子进程，并把输出重定向到日志文件。

- `sleep 2`
  等 MuJoCo 和 ROS2 控制桥先起来，再打印启动信息。

- `tail -f`
  让脚本前台一直显示 3 个日志。按 `Ctrl+C` 时 `trap` 触发，停止整栈。

## 4. 三个子进程

### 4.1 MuJoCo 仿真

脚本实际执行：

```bash
cd ~/pnd_adam_ros2_publish/pnd_mujoco/simulate_python
python3 pnd_mujoco.py
```

它读取 `config.py`，加载 `ROBOT_SCENE` 指定的 MJCF 场景，启动 MuJoCo 窗口，并通过 DDS 发布 `rt/lowstate`、`rt/handstate`，接收 `rt/lowcmd`、`rt/handcmd`。

### 4.2 ROS2 控制层

脚本实际执行：

```bash
cd ~/pnd_adam_ros2_publish
ros2 launch pnd_ros2_control pnd_control.launch.py
```

这个 launch 会启动 3 个节点：

- `pnd_ros2_control_bridge`
  连接 DDS 和 ROS2，实现空闲保位、低增益 PD 控制、手部控制，并创建 MoveIt 需要的 5 个 FollowJointTrajectory Action Server。
- `pnd_joint_state_publisher`
  把 DDS LowState/HandState 发布成 `/joint_states`。
- `robot_state_publisher`
  读 URDF 和 `/joint_states`，发布 TF。

### 4.3 MoveIt2 + RViz

脚本实际执行：

```bash
cd ~/pnd_adam_ros2_publish
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

这个 launch 启动：

- `move_group`
  MoveIt 规划服务、控制器管理、OMPL 规划。
- `rviz2`
  加载 `moveit.rviz`，提供 MotionPlanning 面板和交互标记。

`start_robot_state_publisher:=false` 是因为 ROS2 控制层已经启动了 robot_state_publisher，避免重复。

## 5. 数据流

```text
MuJoCo
  -> DDS LowState/HandState
  -> pnd_joint_state_publisher
  -> /joint_states
  -> robot_state_publisher
  -> /tf

MoveIt/RViz
  -> FollowJointTrajectory
  -> pnd_ros2_control_bridge
  -> DDS LowCmd/HandCmd
  -> MuJoCo
```

## 6. 等价拆分启动

不想用一键脚本时，开 3 个终端。

终端 1：

```bash
cd ~/pnd_adam_ros2_publish/pnd_mujoco/simulate_python
python3 pnd_mujoco.py
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
cd ~/pnd_adam_ros2_publish
source install/setup.bash
ros2 launch pnd_ros2_control pnd_control.launch.py
```

终端 3：

```bash
source /opt/ros/humble/setup.bash
cd ~/pnd_adam_ros2_publish
source install/setup.bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

## 7. 启动后验证

```bash
ros2 node list
ros2 action list
ros2 topic echo /joint_states --once
tail -f /tmp/pnd_mujoco.log /tmp/pnd_ros2_control.log /tmp/pnd_moveit.log
```

正常时日志应出现：

```text
LowState received; control bridge is active
Idle hold active: kp=5.0, kd=1.0
You can start planning now!
```

## 8. 修改场景

当前仿真场景不是命令行参数，而是改 `config.py`：

```bash
nano ~/pnd_adam_ros2_publish/pnd_mujoco/simulate_python/config.py
```

找到：

```python
ROBOT_SCENE = "adam_proximity_scene.xml"
```

改成其他场景文件，例如：

```python
ROBOT_SCENE = "adam_pro_pulse_scene.xml"
```

也可以直接执行：

```bash
sed -i 's/^ROBOT_SCENE = .*/ROBOT_SCENE = "adam_pro_pulse_scene.xml"/' ~/pnd_adam_ros2_publish/pnd_mujoco/simulate_python/config.py
```

改完必须重新执行 `start_pnd_adam.sh`。

## 9. 编译

当前主要修改的包有 3 个：

```bash
cd ~/pnd_adam_ros2_publish
source /opt/ros/humble/setup.bash

colcon build --packages-select pnd_adam_description
colcon build --packages-select pnd_ros2_control
colcon build --packages-select pnd_adam_moveit_config

source install/setup.bash
```

也可以一次编译：

```bash
cd ~/pnd_adam_ros2_publish
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description pnd_ros2_control pnd_adam_moveit_config
source install/setup.bash
```

## 10. 常用修改位置

- 空闲保位增益：
  `~/pnd_adam_ros2_publish/src/pnd_ros2_control/launch/pnd_control.launch.py`
  修改 `idle_kp`、`idle_kd`。

- 关节映射、电机顺序、默认 Kp/Kd：
  `~/pnd_adam_ros2_publish/src/pnd_ros2_control/config/pnd_joint_registry.yaml`

- MoveIt 关节限位：
  `~/pnd_adam_ros2_publish/src/pnd_adam_moveit_config/config/joint_limits.yaml`

- MoveIt IK 配置：
  `~/pnd_adam_ros2_publish/src/pnd_adam_moveit_config/config/kinematics.yaml`

- MoveIt 控制器映射：
  `~/pnd_adam_ros2_publish/src/pnd_adam_moveit_config/config/moveit_controllers.yaml`

## 11. 重新生成 URDF/SRDF

如果修改了 MuJoCo 官方模型，需要重新生成 URDF：

```bash
cd ~/pnd_adam_ros2_publish/src/pnd_adam_description

python3 scripts/generate_adam_pro_urdf.py
python3 scripts/generate_collision_urdf.py
python3 scripts/generate_full_srdf_collision_matrix.py
```

然后重新编译：

```bash
cd ~/pnd_adam_ros2_publish
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description pnd_adam_moveit_config
source install/setup.bash
```

## 12. 单独启动命令

只启动 URDF/TF 显示：

```bash
source /opt/ros/humble/setup.bash
source ~/pnd_adam_ros2_publish/install/setup.bash
ros2 launch pnd_adam_description display.launch.py
```

只启动 MoveIt/RViz，需要 MuJoCo 和 ROS2 控制桥已在运行：

```bash
source /opt/ros/humble/setup.bash
source ~/pnd_adam_ros2_publish/install/setup.bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

临时调整空闲增益单独启动：

```bash
source /opt/ros/humble/setup.bash
source ~/pnd_adam_ros2_publish/install/setup.bash
ros2 launch pnd_ros2_control pnd_control.launch.py \
  idle_kp:=8.0 idle_kd:=1.5
```

## 13. 注意

- `~/pnd_adam_ros2_publish/run_publisher.sh` 是旧版假关节状态发布脚本，不要和 MuJoCo 同时运行，否则会往 `/joint_states` 发全零数据覆盖真实仿真状态。
- 修改完代码后必须重新 `colcon build`，再启动。
- 修改场景、关节映射、URDF 或 MoveIt 配置后，需要重新启动整栈。
