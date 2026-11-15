# pnd_adam_moveit_config · MoveIt2 规划与执行配置

本包将 PND Adam Pro 的机器人描述、SRDF、OMPL、双臂 KDL IK 和轨迹控制器连接到 MoveIt2。RViz 规划得到的轨迹通过 `FollowJointTrajectory` Action 发给 Python ROS2–DDS 控制桥，再驱动 MuJoCo。

## 启动

首次安装与完整启动见[项目首页](../../README.md)。使用一键启动脚本时，本包已自动启动。

如需拆分运行，先启动 MuJoCo 与 `pnd_control.launch.py`，然后在**仓库根目录**另开终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false
```

`pnd_control.launch.py` 已启动机器人状态发布器，所以这里必须关闭重复实例。仅在没有其他状态发布器时，才使用默认的 `start_robot_state_publisher:=true`。

无需 RViz 时添加 `start_rviz:=false`：

```bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_robot_state_publisher:=false start_rviz:=false
```

`use_sim_time` 默认 `false`；只有在完整链路提供 `/clock` 时才启用仿真时间。

## 规划组与控制器

[`srdf/pnd_adam.srdf`](srdf/pnd_adam.srdf) 定义身体、双臂、双手和头部等组；[`config/kinematics.yaml`](config/kinematics.yaml) 当前仅为 `left_arm`、`right_arm` 配置 KDL IK。头部和手部没有笛卡尔 IK，不会因此自动获得 RViz 末端交互标记。

[`config/moveit_controllers.yaml`](config/moveit_controllers.yaml) 配置以下 **5 个** Action 接口，均由同一个 [`pnd_ros2_control_bridge`](../pnd_ros2_control/pnd_ros2_control/pnd_ros2_control_bridge.py) 节点实现：

| 控制器 | Action 名称 | 对应关节 |
|---|---|---:|
| `body_controller` | `/body_controller/follow_joint_trajectory` | 31 |
| `left_arm_controller` | `/left_arm_controller/follow_joint_trajectory` | 7 |
| `right_arm_controller` | `/right_arm_controller/follow_joint_trajectory` | 7 |
| `left_hand_controller` | `/left_hand_controller/follow_joint_trajectory` | 6 |
| `right_hand_controller` | `/right_hand_controller/follow_joint_trajectory` | 6 |

身体组包含双臂关节，各组并非互不重叠。Action 接口与配置文件存在不等于所有组均已完成运行验收；原项目记录已验证身体、左臂、右臂的 Plan & Execute，双手独立轨迹仍列为后续验证任务，见[项目状态](../../PROJECT_STATUS.md)。

## 关键配置

| 文件 | 用途 |
|---|---|
| [`config/kinematics.yaml`](config/kinematics.yaml) | 双臂 KDL IK |
| [`config/ompl_planning.yaml`](config/ompl_planning.yaml) | 默认 OMPL 规划管线与请求适配器 |
| [`config/joint_limits.yaml`](config/joint_limits.yaml) | MoveIt 规划使用的关节限位 |
| [`config/moveit_controllers.yaml`](config/moveit_controllers.yaml) | 控制器、关节列表和 Action 命名 |
| [`config/moveit.rviz`](config/moveit.rviz) | RViz MotionPlanning 显示配置 |
| [`launch/moveit_bringup.launch.py`](launch/moveit_bringup.launch.py) | 启动 `move_group`、可选 RViz 和状态发布器 |

SRDF 同时保存在 [`pnd_adam_description/srdf/`](../pnd_adam_description/srdf/pnd_adam.srdf) 与本包中，修改时需要同步。URDF 的简化碰撞几何和 SRDF 的禁用碰撞对会影响规划结果，应结合[碰撞几何说明](../pnd_adam_description/urdf/pnd_adam_collision_geometry.md)核查。

修改后在仓库根目录重新编译并重启：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description pnd_ros2_control pnd_adam_moveit_config
source install/setup.bash
```

[返回项目首页](../../README.md) · [系统架构](../../ARCHITECTURE.md) · [命令与排错](../../PROJECT_COMMANDS.md)
