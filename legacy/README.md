# Legacy / Deprecated

此目录保留旧版官方 demo、测试脚本和早期 ROS2 包，仅作参考，不参与当前 MuJoCo + DDS + ROS2 + MoveIt2 控制链路。

## 内容

```text
legacy/
├── build_old_demo.sh
├── run_publisher.sh
├── run_subscriber.sh
├── README_OLD_OFFICIAL.md
└── src/
    ├── pnd_joint_state_demo/
    ├── robot_state_publisher/
    └── robot_state_subscriber/
```

## 不要运行

```bash
legacy/run_publisher.sh
```

它是旧版假 `/joint_states` 发布脚本。当前唯一正式的关节状态发布节点是：

```text
src/pnd_ros2_control/pnd_ros2_control/pnd_joint_state_publisher.py
```

如果 MuJoCo 控制链路正在运行，再运行旧版发布脚本会覆盖真实关节状态。
