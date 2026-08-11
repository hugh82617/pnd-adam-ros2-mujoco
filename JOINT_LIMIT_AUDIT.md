# JOINT_LIMIT_AUDIT

来源：

- MuJoCo: `pnd_mujoco/pnd_robots/adam_pro/adam_pro.xml`
- URDF: `src/pnd_adam_description/urdf/pnd_adam.urdf`
- MoveIt: `src/pnd_adam_moveit_config/config/joint_limits.yaml`
- 官方初始 body pose: `0`

| Joint | MuJoCo range | URDF range | MoveIt range | Initial | Conflict |
|---|---|---|---|---|---|
| head_pitch_joint | -0.873 .. 0.873 | -0.873 .. 0.873 | -0.873 .. 0.873 | 0 | ok |
| head_yaw_joint | -1.571 .. 1.571 | -1.571 .. 1.571 | -1.571 .. 1.571 | 0 | ok |
| left_ankle_pitch_joint | -1 .. 0.35 | -1 .. 0.35 | -1 .. 0.35 | 0 | ok |
| left_ankle_roll_joint | -0.3491 .. 0.3491 | -0.3491 .. 0.3491 | -0.3491 .. 0.3491 | 0 | ok |
| left_elbow_joint | -2.496 .. 0.209 | -2.496 .. 0.209 | -2.496 .. 0.209 | 0 | ok |
| left_hip_pitch_joint | -2.164 .. 2.164 | -2.164 .. 2.164 | -2.164 .. 2.164 | 0 | ok |
| left_hip_roll_joint | -0.733 .. 1.605 | -0.733 .. 1.605 | -0.733 .. 1.605 | 0 | ok |
| left_hip_yaw_joint | -0.785 .. 0.785 | -0.785 .. 0.785 | -0.785 .. 0.785 | 0 | ok |
| left_knee_pitch_joint | 0.052 .. 2.391 | 0.052 .. 2.391 | 0.052 .. 2.391 | 0 | initial 0 < MoveIt min 0.052 |
| left_shoulder_pitch_joint | -3.613 .. 2.042 | -3.613 .. 2.042 | -3.613 .. 2.042 | 0 | ok |
| left_shoulder_roll_joint | -0.628 .. 2.793 | -0.628 .. 2.793 | -0.628 .. 2.793 | 0 | ok |
| left_shoulder_yaw_joint | -2.583 .. 2.583 | -2.583 .. 2.583 | -2.583 .. 2.583 | 0 | ok |
| left_wrist_pitch_joint | -0.96 .. 0.96 | -0.96 .. 0.96 | -0.96 .. 0.96 | 0 | ok |
| left_wrist_roll_joint | -0.96 .. 0.96 | -0.96 .. 0.96 | -0.96 .. 0.96 | 0 | ok |
| left_wrist_yaw_joint | -2.67 .. 2.67 | -2.67 .. 2.67 | -2.67 .. 2.67 | 0 | ok |
| right_ankle_pitch_joint | -1 .. 0.35 | -1 .. 0.35 | -1 .. 0.35 | 0 | ok |
| right_ankle_roll_joint | -0.3491 .. 0.3491 | -0.3491 .. 0.3491 | -0.3491 .. 0.3491 | 0 | ok |
| right_elbow_joint | -2.496 .. 0.209 | -2.496 .. 0.209 | -2.496 .. 0.209 | 0 | ok |
| right_hip_pitch_joint | -2.164 .. 2.164 | -2.164 .. 2.164 | -2.164 .. 2.164 | 0 | ok |
| right_hip_roll_joint | -1.605 .. 0.733 | -1.605 .. 0.733 | -1.605 .. 0.733 | 0 | ok |
| right_hip_yaw_joint | -0.785 .. 0.785 | -0.785 .. 0.785 | -0.785 .. 0.785 | 0 | ok |
| right_knee_pitch_joint | 0.052 .. 2.391 | 0.052 .. 2.391 | 0.052 .. 2.391 | 0 | initial 0 < MoveIt min 0.052 |
| right_shoulder_pitch_joint | -3.613 .. 2.042 | -3.613 .. 2.042 | -3.613 .. 2.042 | 0 | ok |
| right_shoulder_roll_joint | -2.793 .. 0.628 | -2.793 .. 0.628 | -2.793 .. 0.628 | 0 | ok |
| right_shoulder_yaw_joint | -2.583 .. 2.583 | -2.583 .. 2.583 | -2.583 .. 2.583 | 0 | ok |
| right_wrist_pitch_joint | -0.96 .. 0.96 | -0.96 .. 0.96 | -0.96 .. 0.96 | 0 | ok |
| right_wrist_roll_joint | -0.96 .. 0.96 | -0.96 .. 0.96 | -0.96 .. 0.96 | 0 | ok |
| right_wrist_yaw_joint | -2.67 .. 2.67 | -2.67 .. 2.67 | -2.67 .. 2.67 | 0 | ok |
| waist_pitch_joint | -0.663 .. 1.361 | -0.663 .. 1.361 | -0.663 .. 1.361 | 0 | ok |
| waist_roll_joint | -0.279 .. 0.279 | -0.279 .. 0.279 | -0.279 .. 0.279 | 0 | ok |
| waist_yaw_joint | -0.829 .. 0.829 | -0.829 .. 0.829 | -0.829 .. 0.829 | 0 | ok |

## 冲突

- `left_knee_pitch_joint`: initial 0 < MoveIt min 0.052
- `right_knee_pitch_joint`: initial 0 < MoveIt min 0.052
