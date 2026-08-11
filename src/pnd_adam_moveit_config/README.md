# pnd_adam_moveit_config

MoveIt2 configuration for the PND Adam Pro body and arm groups.

The SRDF is maintained in both `pnd_adam_description/srdf/pnd_adam.srdf` and
this package's `srdf/pnd_adam.srdf`. Keep the copy in sync when editing the
canonical description file.

## Launch

With MuJoCo, the ROS2 bridge, and `/joint_states` running:

```bash
source /opt/ros/humble/setup.bash
source ~/pnd_adam_ros2_publish/install/setup.bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py
```

Useful overrides:

```bash
ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
  start_rviz:=false
```

## Controllers

`moveit_controllers.yaml` maps MoveIt controller names to the Python bridge
action servers:

- `body_controller/follow_joint_trajectory`
- `left_arm_controller/follow_joint_trajectory`
- `right_arm_controller/follow_joint_trajectory`

All three are implemented by the same `pnd_ros2_control_bridge` node.
