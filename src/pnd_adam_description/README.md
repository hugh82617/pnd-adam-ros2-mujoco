# pnd_adam_description · 机器人模型与 TF

本包保存 PND Adam Pro 的 URDF、网格、关节限位和 TF 启动文件，为 RViz 与 MoveIt2 提供同一套机器人描述。

当前运行入口读取 [`urdf/pnd_adam.urdf`](urdf/pnd_adam.urdf)，其中有 **43 个 revolute 关节：31 个身体关节和 12 个手部控制关节**，另有固定连接。手部控制关节是 ROS 侧的简化表达；MuJoCo 中的手指关节通过 [`hand_mapping.yaml`](../../pnd_mujoco/simulate_python/hand_mapping.yaml) 映射，不能把两者的物理关节数量直接等同。

## 文件索引

| 文件 / 目录 | 用途 |
|---|---|
| [`urdf/pnd_adam.urdf`](urdf/pnd_adam.urdf) | 当前 launch 加载的完整 URDF，含简化碰撞几何和手部控制关节 |
| [`urdf/adam_pro.urdf.xacro`](urdf/adam_pro.urdf.xacro) | 从 Adam Pro MJCF 转换得到的描述中间文件 |
| [`meshes/`](meshes/) | 机器人和场景网格，供 URDF 与 MuJoCo 使用 |
| [`config/joint_limits.yaml`](config/joint_limits.yaml) | 描述侧关节限位；MoveIt 使用其自身包内的限位配置 |
| [`srdf/pnd_adam.srdf`](srdf/pnd_adam.srdf) | 语义描述副本，需与 MoveIt 配置包中的副本保持同步 |
| [`launch/display.launch.py`](launch/display.launch.py) | 启动系统 `robot_state_publisher` 与静态 `world → base_link` TF |
| [`scripts/`](scripts/) | MJCF 转换、碰撞几何和 SRDF 辅助生成工具 |

## 编译

在**仓库根目录**执行，环境安装见[项目首页](../../README.md)：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description
source install/setup.bash
```

## 在完整仿真链路中使用

完整链路使用 `bash start_pnd_adam.sh`。其中 `pnd_control.launch.py` 已通过本包的 `robot_state_publisher.launch.py` 启动机器人状态发布器与静态 TF，因此不要再单独重复启动。正式 `/joint_states` 来自 [`pnd_joint_state_publisher`](../pnd_ros2_control/pnd_ros2_control/pnd_joint_state_publisher.py)，它读取 MuJoCo 的 DDS 状态。

```text
MuJoCo → DDS LowState / HandState → pnd_joint_state_publisher
       → /joint_states → robot_state_publisher → TF → RViz / MoveIt2
```

若已有 `/joint_states`，但尚未启动任何 `robot_state_publisher`，可以单独运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pnd_adam_description display.launch.py
```

该命令**不启动 MuJoCo、DDS 状态节点或 RViz**。需要可视化时另开终端运行 `rviz2`，添加 `RobotModel`，并将 Fixed Frame 设为 `world`。

## 模型维护与边界

模型源文件位于 [`pnd_mujoco/pnd_robots/adam_pro/adam_pro.xml`](../../pnd_mujoco/pnd_robots/adam_pro/adam_pro.xml)。历史转换脚本仍包含旧工作区布局的默认路径；重新生成时应检查并显式指定 `--source`、`--mesh-source`、`--output-dir`。不要直接覆盖当前运行 URDF 后假定与手部映射、碰撞几何及 SRDF 一致。

MoveIt 中的固定基座 TF 不是 MuJoCo 自由基座位姿反馈。当前说明对应关节级规划与执行，不表示实现了步态、全身平衡或浮动基座控制。膝关节初始零位与限位之间的冲突见[关节限位审计](../../JOINT_LIMIT_AUDIT.md)。

[`legacy/run_publisher.sh`](../../legacy/run_publisher.sh) 使用旧版假状态发布器，不应与正式链路同时运行，否则 `/joint_states` 会出现多个来源。

[返回项目首页](../../README.md) · [系统架构](../../ARCHITECTURE.md) · [MoveIt2 配置](../pnd_adam_moveit_config/README.md)
