# 第三方来源与许可记录

本项目在 PNDbotics 的模型、仿真器和 Python DDS SDK 基础上，整合 ROS2 Humble、MoveIt2、RViz 及控制桥。下表记录当前仓库能够直接核对的来源和许可文件，整理日期为 2026-11-15。

## 仓库内组件

| 内容 | 来源线索 | 当前许可记录 |
|---|---|---|
| [`pnd_mujoco/`](../pnd_mujoco/) | [上游 pndbotics/pnd_mujoco](https://github.com/pndbotics/pnd_mujoco)，目录内 README 保留上游介绍；当前目录含本项目改动 | [`pnd_mujoco/LICENSE`](../pnd_mujoco/LICENSE) 为完整 BSD 3-Clause 文本，版权标识为 2025 PNDbotics AI CO., LTD. |
| [`pnd_sdk_python/`](../pnd_sdk_python/) | [上游 pndbotics/pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python)；`setup.py` 另列 PNDbotics GitLab 源地址 | `setup.py` 声明 `BSD-3-Clause`，README 也作相同声明，但该目录当前没有 README 所链接的 `LICENSE` 文件 |
| Adam Pro MJCF、URDF 与网格 | [`pnd_mujoco/pnd_robots/adam_pro/`](../pnd_mujoco/pnd_robots/adam_pro/) 及 [`src/pnd_adam_description/`](../src/pnd_adam_description/)；转换脚本记录从 Adam Pro MJCF 生成 ROS 描述的过程 | 已有 `pnd_mujoco/LICENSE` 与包内 BSD 声明；复制到 `meshes/` 的资产未逐文件附加来源清单，不能据此额外推定所有网格拥有独立、相同的授权证明 |
| ROS2 描述、控制桥与 MoveIt 配置 | [`src/`](../src/) 下三个当前 ROS2 包 | `package.xml` 声明 `BSD-3-Clause`；控制桥 `setup.py` 同样声明 BSD，根目录尚无覆盖整个仓库的统一 `LICENSE` 文件 |
| 人体前臂网格 | [`meshes/human_arm/untitled.obj`](../src/pnd_adam_description/meshes/human_arm/untitled.obj) 与配套 MTL，供场景使用 | 文件头仅记录 Blender 导出信息，当前仓库未提供原始作者、下载地址或单独许可文本；Blender 导出信息不能替代资产来源与许可记录 |
| 历史示例 | [`legacy/`](../legacy/) 与各上游目录内示例 | 保留各文件原有声明；不参与默认启动链路 |

当前仓库没有记录所复制上游目录的精确提交号，不应把上游现行版本号当成本地快照版本。SDK README 中的版本徽章、版本表和 `setup.py` 版本字段也并不完全一致。

## 外部运行依赖

系统与 Python 依赖由安装脚本及包管理器安装，没有因为被使用而统一改用本项目许可。版本要求见 [`requirements.txt`](../requirements.txt)、[`scripts/setup_environment.sh`](../scripts/setup_environment.sh) 和各 ROS2 包的 `package.xml`。

主要依赖包括 [ROS2](https://github.com/ros2)、[MoveIt2](https://github.com/moveit/moveit2)、[MuJoCo](https://github.com/google-deepmind/mujoco)、[Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) 及其 Python 接口。其许可和版权应以实际安装版本所附文件为准。

## 待补齐的来源材料

- 为复制的上游 SDK 补齐与实际快照对应的完整许可文本和提交号。
- 明确本项目自有改动的版权归属及许可范围，再补充根目录许可文件。
- 为机器人网格及人体前臂资产建立来源、作者、版本和许可清单。

本次整理保留现有版权和许可声明，没有新增未经核实的统一授权，也没有把本仓库描述为 PNDbotics 官方发布。

[返回项目首页](../README.md)
