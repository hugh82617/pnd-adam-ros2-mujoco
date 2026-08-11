# PROJECT_STATUS

当前项目状态：**可运行冻结版本候选 v0.1.0**。

## 已完成的阶段

| Phase | 内容 | 状态 |
|---|---|---|
| Phase 1 | PND SDK / MuJoCo Adam Pro | DONE |
| Phase 2 | DDS LowState / LowCmd / HandState / HandCmd | DONE |
| Phase 3 | ROS2 Adapter Layer | DONE |
| Phase 4 | JointState / TF | DONE |
| Phase 5 | MoveIt2 Planning | DONE |
| Phase 6 | MoveIt Execute 驱动 MuJoCo | DONE |

## 当前已验证

- 43 JointState：31 body + 12 hand
- `world -> base_link -> torso -> arms/legs/head/hands` TF
- RViz MoveIt MotionPlanning
- OMPL 规划
- left_arm Plan & Execute
- right_arm Plan & Execute
- body Plan & Execute
- DDS LowCmd/HandCmd 下发
- DDS LowState/HandState 回读
- FollowJointTrajectory Action
- 空闲低增益保位
- LowState watchdog / DDS 超时处理

## 当前已知问题

1. 膝关节官方初始 `q=0` 低于 MoveIt `joint_limits.yaml` 最小值 `0.052`，MoveIt 会修正到最小值。
2. `head` 没有 IK 配置，RViz 没有头部交互标记。
3. `left_hand` / `right_hand` 没有 IK，更适合使用 joint-space trajectory。
4. `pnd_mujoco/simulate_python/ros2_adapter` 是 MuJoCo 端旧 ROS2 输出参考，当前正式 `joint_states` 由 `pnd_ros2_control` 发布。
5. `legacy/` 保存旧版 demo 和脚本，不参与当前控制闭环。

## 后续版本路线

- v0.2.0：手部 12 DOF joint-space trajectory
- v0.3.0：IMU
- v0.4.0：Proximity Sensor
- v0.5.0：Isaac Sim 替换 MuJoCo，保留 ROS2 接口
- v1.0.0：Sim2Real
