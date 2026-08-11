# Architecture

## 系统架构

```text
                RViz
                 │
            MoveIt2
                 │
    FollowJointTrajectory
                 │
                 ▼
    pnd_ros2_control_bridge
                 │
          Joint Registry
                 │
        LowCmd / HandCmd
                 │
                DDS
                 │
              MuJoCo
                 │
        LowState / HandState
                 │
                DDS
                 │
    pnd_joint_state_publisher
                 │
           /joint_states
                 │
      robot_state_publisher
                 │
                 TF
```

## 四条路径

### Control Path

```text
MoveIt / RViz
  -> FollowJointTrajectory
  -> pnd_ros2_control_bridge
  -> LowCmd / HandCmd
  -> DDS
  -> MuJoCo actuator
```

### Feedback Path

```text
MuJoCo sensor
  -> LowState / HandState
  -> DDS
  -> pnd_joint_state_publisher
  -> /joint_states
  -> robot_state_publisher
  -> TF
```

### Planning Path

```text
RViz MotionPlanning
  -> move_group
  -> OMPL / KDL IK
  -> trajectory
  -> controller selection
```

### State Path

```text
DDS LowState
  -> pnd_joint_state_publisher
  -> sensor_msgs/JointState
  -> robot_state_publisher
  -> tf2
  -> RViz / MoveIt current state
```

## 包职责

| Package | 职责 |
|---|---|
| `pnd_mujoco/simulate_python` | MuJoCo 仿真、DDS 状态发布、DDS 命令接收 |
| `pnd_sdk_python` | PND Python DDS SDK |
| `pnd_adam_description` | URDF、mesh、joint_limits、TF launch |
| `pnd_adam_moveit_config` | SRDF、IK、joint limits、controllers、MoveIt launch |
| `pnd_ros2_control` | ROS2-DDS bridge、JointState、FollowJointTrajectory、idle hold |

## 当前关键文件

- MuJoCo 入口：`pnd_mujoco/simulate_python/pnd_mujoco.py`
- ROS2 控制桥：`src/pnd_ros2_control/pnd_ros2_control/pnd_ros2_control_bridge.py`
- JointState：`src/pnd_ros2_control/pnd_ros2_control/pnd_joint_state_publisher.py`
- Joint Registry：`src/pnd_ros2_control/config/pnd_joint_registry.yaml`
- MoveIt：`src/pnd_adam_moveit_config`
