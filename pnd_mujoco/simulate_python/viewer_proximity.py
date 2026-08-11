import time
import numpy as np

import mujoco
import mujoco.viewer

import config
from proximity_sensor import ProximityRaySensor


# ==========================================
# 加载模型
# ==========================================

model = mujoco.MjModel.from_xml_path(
    config.ROBOT_SCENE
)

data = mujoco.MjData(model)


# ==========================================
# 创建接近觉传感器
# ==========================================

sensor = ProximityRaySensor(
    sensor_range=0.5
)

sensor.init_id(model)


# ==========================================
# 获取Site ID
# ==========================================

site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "proximity_sensor"
)

print("Sensor site id:", site_id)


# ==========================================
# 获取右肘Joint ID
# ==========================================

elbow_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_JOINT,
    "elbow_Right"
)

print("Elbow joint id:", elbow_id)


# qpos中的索引
qpos_addr = model.jnt_qposadr[elbow_id]

print("Elbow qpos addr:", qpos_addr)


# ==========================================
# 初始化
# ==========================================

mujoco.mj_forward(
    model,
    data
)


# ==========================================
# 时间变量
# ==========================================

t = 0.0
dt = 0.01


# ==========================================
# 打开Viewer
# ==========================================

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    while viewer.is_running():

        # ----------------------------
        # 更新时间
        # ----------------------------

        t += dt

        # ----------------------------
        # 右肘周期摆动
        # ----------------------------

        data.qpos[qpos_addr] = (
            -1.0
            + 0.5 * np.sin(2.0 * t)
        )

        # ----------------------------
        # 更新运动学（不进行动力学仿真）
        # ----------------------------

        mujoco.mj_forward(
            model,
            data
        )

        # ----------------------------
        # 测量接近觉
        # ----------------------------

        result = sensor.measure(
            model,
            data
        )

        # ----------------------------
        # 打印结果
        # ----------------------------

        print(
            "\rProximity:",
            np.round(result, 3),
            end=""
        )

        # ----------------------------
        # 更新Viewer
        # ----------------------------

        viewer.sync()

        time.sleep(dt)
