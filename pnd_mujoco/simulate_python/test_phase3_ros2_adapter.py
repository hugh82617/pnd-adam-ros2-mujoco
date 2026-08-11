from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, JointState

from pnd_adapter import load_hand_mapping, load_motor_mapping
from pndbotics_sdk_py.idl.default import (
    pnd_adam_msg_dds__HandState_,
    pnd_adam_msg_dds__LowState_,
)
from ros2_adapter.hand_state_publisher import build_hand_state
from ros2_adapter.imu_publisher import build_imu_msg
from ros2_adapter.joint_state_publisher import build_joint_state


def test_yaml_names_are_complete():
    motor_cfg = load_motor_mapping()
    hand_cfg = load_hand_mapping()
    assert len(motor_cfg["sdk_to_mujoco"]) == 31
    assert len(motor_cfg["sdk_motor_names"]) == 31
    assert len(motor_cfg["ros_joint_names"]) == 31
    assert len(hand_cfg["hand_position_groups"]) == 12


def test_joint_state_builder():
    motor_cfg = load_motor_mapping()
    joint_names = [
        motor_cfg["ros_joint_names"][i]
        for i in sorted(motor_cfg["sdk_to_mujoco"])
    ]
    low_state = pnd_adam_msg_dds__LowState_(31)
    for i, motor in enumerate(low_state.motor_state):
        motor.q = 0.1 * i
        motor.dq = 0.01 * i
        motor.tau_est = 0.5 * i

    msg = build_joint_state(Time(sec=1, nanosec=2), joint_names, low_state)

    assert isinstance(msg, JointState)
    assert len(msg.name) == 31
    assert len(msg.position) == 31
    assert len(msg.velocity) == 31
    assert len(msg.effort) == 31
    assert msg.name[12] == "waist_yaw_joint"
    assert msg.position[15] == 1.5


def test_imu_builder():
    low_state = pnd_adam_msg_dds__LowState_(31)
    low_state.imu_state.quaternion[0] = 1.0
    low_state.imu_state.quaternion[1] = 0.1
    low_state.imu_state.quaternion[2] = 0.2
    low_state.imu_state.quaternion[3] = 0.3
    low_state.imu_state.gyroscope[0] = 0.4
    low_state.imu_state.gyroscope[1] = 0.5
    low_state.imu_state.gyroscope[2] = 0.6
    low_state.imu_state.accelerometer[0] = 0.7
    low_state.imu_state.accelerometer[1] = 0.8
    low_state.imu_state.accelerometer[2] = 0.9

    msg = build_imu_msg(Time(sec=2), low_state)

    assert isinstance(msg, Imu)
    assert msg.header.frame_id == "imu_link"
    assert msg.orientation.w == 1.0
    assert msg.angular_velocity.z == 0.6
    assert msg.linear_acceleration.z == 0.9


def test_hand_state_builder():
    hand_cfg = load_hand_mapping()
    hand_names = [group["name"] for group in hand_cfg["hand_position_groups"]]
    hand_state = pnd_adam_msg_dds__HandState_()
    for i in range(12):
        hand_state.position[i] = i * 100

    msg = build_hand_state(Time(sec=3), hand_names, hand_state)

    assert isinstance(msg, JointState)
    assert len(msg.name) == 12
    assert msg.position[4] == 400.0


def _run_all():
    tests = [
        value
        for name, value in sorted(globals().items())
        if name.startswith("test_") and callable(value)
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    _run_all()
