# pnd_adam_description

URDF/Xacro and TF description for PND Adam Pro.

The URDF is generated from `pnd_mujoco/pnd_robots/adam_pro/adam_pro.xml`.
It contains the 31 controlled PND joints with ROS-style names, plus fixed
connections for the finger links.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/pnd_adam_ros2_publish
colcon build --packages-select pnd_adam_description
source install/setup.bash
```

## Run

With `pnd_mujoco.py` publishing `/joint_states`:

```bash
ros2 launch pnd_adam_description display.launch.py
```

Use RViz2 with the `RobotModel` display to inspect the synchronized model.

Do not run `pnd_adam_ros2_publish/run_publisher.sh` while `pnd_mujoco.py` is
running: that script starts the old fake `joint_state_publisher`, which will
overwrite `/joint_states` with zeros.
