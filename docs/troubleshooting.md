# 常见问题排查

[返回项目首页](../README.md) · [运行命令](../PROJECT_COMMANDS.md) · [验证边界](../PROJECT_STATUS.md)

## 启动前提示缺少 ROS 或 install/setup.bash

先确认使用 Ubuntu 22.04、ROS 2 Humble 和系统 Python 3.10。安装与编译完成后再启动：

```bash
bash scripts/setup_environment.sh
bash start_pnd_adam.sh
```

当前启动脚本按脚本自身位置定位仓库，克隆目录无需命名为旧的 `pnd_adam_ros2_publish`。安装脚本要求 ROS 已存在；编译失败时，查看首个错误再重新编译。

## MuJoCo 报 No module named rclpy

仿真入口即使关闭旧 ROS 输出，仍会导入 `rclpy`。整栈脚本已提前加载 ROS 环境；手动启动时也需要加载：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cd pnd_mujoco/simulate_python
python3 pnd_mujoco.py
```

使用系统 Python 3.10；其他 Python 版本或隔离环境可能无法加载 Humble 的二进制模块。

## pip 提示找不到 CycloneDDS

先查看实际报错和 [SDK 的安装说明](../pnd_sdk_python/README.md)。Python 绑定固定为 `cyclonedds==0.10.2`；若当前平台没有匹配的预编译包，需要准备匹配的 CycloneDDS 原生库，并按 SDK 说明设置 `CYCLONEDDS_HOME` 后重试。

不要为了绕过安装错误直接升级绑定版本；DDS 消息与底层库组合需要验证。

## MuJoCo / RViz 窗口无法打开

确认在有图形会话的 Ubuntu 桌面运行，OpenGL 驱动与显示环境可用。当前默认入口会打开 MuJoCo Viewer 和 RViz，没有配置为无显示环境下的整栈启动方式。分别查看 `/tmp/pnd_mujoco.log` 与 `/tmp/pnd_moveit.log`，区分模型加载错误和图形初始化错误。

## 控制桥没有收到 LowState

1. 查看 MuJoCo 日志，确认仿真没有因模型或依赖错误退出。
2. 核对 [config.py](../pnd_mujoco/simulate_python/config.py) 与 [pnd_control.launch.py](../src/pnd_ros2_control/launch/pnd_control.launch.py)：默认均为 DDS domain `1`、网卡 `lo`。
3. 查看 `/tmp/pnd_ros2_control.log`，确认出现 `LowState received; control bridge is active`。

`ROS_DOMAIN_ID` 控制 ROS 发现，不能代替 PND SDK 的 domain 配置。DDS LowState 缺失或写入失败会导致轨迹 Action 中止。

## RViz 状态跳变或 TF 重复

正式 `/joint_states` 应由 `pnd_joint_state_publisher` 发布：

```bash
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
```

停止 `legacy` 假关节状态发布器；保持 MuJoCo 的 `ENABLE_ROS2_OUTPUT=False`。若控制层已经启动，再单独启动 MoveIt 时设置：

```bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

默认 TF 基座固定，不能用它判断机器人在 MuJoCo 中的自由基座位置。

## 没有头部或手部交互标记

当前只有 `left_arm` 与 `right_arm` 配置 KDL IK。先选择双臂中的一个检查规划；头部和手部没有相同的笛卡尔交互入口。手部 SDK 原始位置值还未与 URDF 弧度统一，手部轨迹属于后续验证内容。

## 双膝初始状态超出限位

官方初始身体姿态为 `q=0`，而双膝下界是 `0.052 rad`，详见[关节限位审计](../JOINT_LIMIT_AUDIT.md)。修改模型、初始姿态或 MoveIt 限位前，应同时核对 MJCF、URDF 与规划配置。不要仅消除警告而留下模型与控制接口不一致。

可在仓库根目录重新生成审计表：

```bash
python3 scripts/generate_joint_limit_audit.py
```

## Action 显示成功但动作未准确到位

当前桥主要依据轨迹时长和 DDS 状态返回成功，并未完整执行最终误差与容差验收。结合 MuJoCo、`/joint_states` 和轨迹反馈检查实际位置；Action 的成功结果不能单独作为轨迹精度证明。多组控制器共用轨迹状态，先串行执行验证。

## 提交问题时附哪些信息

请提供 Ubuntu、ROS、Python 与依赖版本，当前 commit，执行命令、期望行为和实际行为，以及三份启动日志中对应的错误。默认日志位于 `/tmp`；设置 `PND_LOG_DIR` 后以自定义目录为准。
