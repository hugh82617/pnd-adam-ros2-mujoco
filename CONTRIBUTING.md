# 参与改进

[返回项目首页](README.md) · [项目状态](PROJECT_STATUS.md)

欢迎提交安装复现、模型映射、轨迹执行、传感器接口和文档改进。

## 报告问题

提交 Issue 时请附：

- 当前 commit、Ubuntu / ROS / Python 版本。
- 从哪个目录执行了哪些命令，以及期望与实际行为。
- `/tmp/pnd_mujoco.log`、`/tmp/pnd_ros2_control.log`、`/tmp/pnd_moveit.log` 中与问题相关的片段。
- 是否修改过 DDS domain、网卡、场景、URDF、关节注册表或控制参数。

## 提交修改

1. 建立分支，保持一次修改围绕一个具体问题。
2. 说明行为如何变化，以及验证环境和结果。
3. 修改关节映射或限位时，同步核对 MJCF、URDF、MoveIt 配置与注册表；修改 SRDF 时留意两个包中的副本。
4. 修改控制路径时，区分静态检查、单元测试和实际仿真验证；没有运行的检查请明确注明。
5. 保留上游版权和许可信息，第三方来源说明见 [docs/third-party-notices.md](docs/third-party-notices.md)。

在已安装 ROS 的 Ubuntu 工作区内，可运行现有包测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select pnd_ros2_control
colcon test-result --verbose
```

文档和脚本改动可先执行 `git diff --check`、`bash -n start_pnd_adam.sh scripts/setup_environment.sh`。测试通过不代表已经完成仿真运动或真机验证。
