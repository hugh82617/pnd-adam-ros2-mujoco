import time
import atexit
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
from pndbotics_sdk_py_bridge import pndSdkBridge, ElasticBand
from pndbotics_ros_bridge import pndRos2Bridge
from pnd_ros2_adapter import PndRos2Adapter

import rclpy
import config


locker = threading.Lock()
pnd_bridge = None
ros_adapter = None


def shutdown():
    global pnd_bridge, ros_adapter
    if pnd_bridge is not None and hasattr(pnd_bridge, "Shutdown"):
        pnd_bridge.Shutdown()
    if ros_adapter is not None and rclpy.ok():
        rclpy.shutdown()
    ros_adapter = None
    pnd_bridge = None


atexit.register(shutdown)

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)
azimuth = mj_model.vis.global_.azimuth
elevation = mj_model.vis.global_.elevation


def set_hand_open_pose(model, data):
    open_pose = {
        "L_pinky_DIP_joint": 1.6,
        "L_pinky_MCP_joint": 1.6,
        "L_ring_DIP_joint": 1.6,
        "L_ring_MCP_joint": 1.6,
        "L_middle_DIP_joint": 1.6,
        "L_middle_MCP_joint": 1.6,
        "L_index_DIP_joint": 1.6,
        "L_index_MCP_joint": 1.6,
        "L_thumb_DIP_joint": 1.0,
        "L_thumb_PIP_joint": 1.0,
        "L_thumb_MCP_joint2": 0.5,
        "L_thumb_MCP_joint1": 1.0,
        "R_pinky_DIP_joint": 1.6,
        "R_pinky_MCP_joint": 1.6,
        "R_ring_DIP_joint": 1.6,
        "R_ring_MCP_joint": 1.6,
        "R_middle_DIP_joint": 1.6,
        "R_middle_MCP_joint": 1.6,
        "R_index_DIP_joint": 1.6,
        "R_index_MCP_joint": 1.6,
        "R_thumb_DIP_joint": 1.0,
        "R_thumb_PIP_joint": 1.0,
        "R_thumb_MCP_joint2": 0.5,
        "R_thumb_MCP_joint1": 1.0,
    }
    for joint_name, position in open_pose.items():
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id >= 0:
            data.qpos[model.jnt_qposadr[joint_id]] = position


set_hand_open_pose(mj_model, mj_data)

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()

    band_attached_link = mj_model.body("torso").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

viewer.cam.azimuth = azimuth
viewer.cam.elevation = elevation
viewer.cam.distance = 3.0
viewer.cam.lookat[:] = [0, 0, 1]

# =========================
# Enable site visualization
# =========================
# 显示 site
viewer.opt.sitegroup[0] = 1

mj_model.opt.timestep = config.SIMULATE_DT

time.sleep(0.2)


def SimulationThread():
    global pnd_bridge, ros_adapter
    ros_adapter = None
    pnd = None

    if config.ENABLE_ROS2_OUTPUT:
        rclpy.init()
        ros_adapter = PndRos2Adapter()
    global mj_data, mj_model

    if config.SDK_TYPE == "ROS2":
        # pnd = pndSdkBridge(mj_model, mj_data)
        ChannelFactoryInitialize(config.DOMAIN_ID)
        pnd = pndRos2Bridge(mj_model, mj_data)
    elif config.SDK_TYPE == "DDS":
        # use python sdk example
        # ChannelFactoryInitialize(config.DOMAIN_ID)

        # use pnd_mujoco open_arm.py example
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        pnd = pndSdkBridge(mj_model, mj_data)

    if ros_adapter is not None and pnd is not None:
        pnd.ros2_adapter = ros_adapter
    pnd_bridge = pnd

    # print(pnd.low_cmd_suber)

    if config.USE_JOYSTICK:
        pnd.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        pnd.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if pnd is not None and hasattr(pnd, "ApplyWatchdog"):
            pnd.ApplyWatchdog()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3])
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    try:
        viewer_thread.start()
        sim_thread.start()
        viewer_thread.join()
        sim_thread.join()
    except KeyboardInterrupt:
        print("shutting down")
        shutdown()
        viewer.close()
