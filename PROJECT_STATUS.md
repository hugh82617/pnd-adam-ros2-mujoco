# 项目状态与路线

[返回项目首页](README.md) · [系统架构](ARCHITECTURE.md) · [问题排查](docs/troubleshooting.md)

**当前阶段：v0.1.0 可运行冻结候选。** 这是仓库现有阶段记录，不代表已经发布 GitHub Release，也不代表完整的产品验收。

## 已有验证记录

以下能力沿用原项目的状态记录，本次主页整理未重新运行 Ubuntu / ROS / MuJoCo 整栈。

| 阶段 | 已有记录 |
|---|---|
| 仿真与 DDS | PND SDK / MuJoCo Adam Pro；LowState / LowCmd / HandState / HandCmd 通信 |
| ROS 状态 | 43 个状态通道：31 body + 12 hand；JointState 与 TF 接入 |
| 运动规划 | RViz MotionPlanning、OMPL 规划、双臂 KDL IK |
| 仿真执行 | `left_arm`、`right_arm`、`body` 的 Plan & Execute |
| 控制桥 | FollowJointTrajectory、空闲低增益保位、LowState watchdog 与 DDS 写入异常处理 |

本次整理完成了说明与实际文件的核对，并修复启动脚本的固定路径、ROS 环境加载和日志目录问题。文档中的定时器频率来自配置，不是实时性能或轨迹精度测试结果。

## 已知问题与限制

| 问题 | 当前状态 / 影响 |
|---|---|
| 双膝初始姿态与关节限位不一致 | 初始 `q=0` 低于 `0.052 rad` 下界，可能触发 MoveIt 起始状态修正；见[限位审计](JOINT_LIMIT_AUDIT.md) |
| 头部与手部缺少 IK | 目前只有双臂配置 KDL；头部/手部不具备同等的笛卡尔交互规划能力 |
| 手部单位未对齐 | SDK 原始位置值直接用于状态/命令，尚未完成 URDF 弧度映射和手部轨迹验证 |
| TF 基座固定 | `world → base_link` 为静态 TF，未同步 MuJoCo 的自由基座运动 |
| 默认弹性悬挂 | 用于调试，不构成自主平衡、站立或行走的验证 |
| Action 到位判断有限 | 主要按轨迹时长加 `0.5 s` 及 DDS 状态返回成功，未按最终关节误差与 trajectory tolerance 严格判定 |
| 多控制器共享轨迹状态 | 五个控制器和一个兼容别名共用桥中轨迹数据，独立并发执行未验证 |
| 自定义 Python 控制实现 | 未实现标准 `ros2_control` 硬件插件或 `controller_manager` |
| 存在历史路径与旧示例 | 部分模型生成工具保留旧工作区默认路径；`legacy` 和 MuJoCo 旧 ROS 输出不参与正式运行 |

## 后续路线

以下版本号沿用原规划，均为待完成目标，没有承诺发布日期。

| 阶段 | 目标 | 需要验证 |
|---|---|---|
| v0.2.0 | 手部 12 通道 joint-space trajectory | 单位映射、限位、状态和命令一致性 |
| v0.3.0 | 正式 ROS IMU 接口 | 坐标系、单位、时间戳与数据链路 |
| v0.4.0 | Proximity Sensor 接入 | 传感器输出、ROS 消息与场景测试 |
| v0.5.0 | Isaac Sim 后端适配 | 保持 ROS 接口并验证模型和控制语义 |
| v1.0.0 | Sim2Real | 真机接口、控制行为与实际硬件验证 |

MuJoCo 场景和适配代码已有部分传感器实现，不等同于上述正式 ROS 集成已经完成。路线之外，优先补齐轨迹误差验证、基座状态同步、并发控制和可重复的 Ubuntu 环境测试。