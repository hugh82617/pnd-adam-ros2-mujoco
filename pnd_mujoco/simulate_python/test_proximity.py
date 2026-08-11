import mujoco
import config

from proximity_sensor import ProximityRaySensor



model=mujoco.MjModel.from_xml_path(
    config.ROBOT_SCENE
)


data=mujoco.MjData(model)



sensor=ProximityRaySensor(
    sensor_range=0.5
)



sensor.init_id(model)



mujoco.mj_forward(
    model,
    data
)



result=sensor.measure(
    model,
    data
)



print(
    "Proximity:",
    result
)
