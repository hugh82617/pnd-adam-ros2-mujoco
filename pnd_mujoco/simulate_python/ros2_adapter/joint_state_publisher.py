"""Build standard sensor_msgs/JointState from PND LowState."""

from typing import Any, List

from sensor_msgs.msg import JointState


def build_joint_state(
    stamp: Any,
    joint_names: List[str],
    low_state: Any,
) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.name = list(joint_names)
    msg.position = [float(m.q) for m in low_state.motor_state]
    msg.velocity = [float(m.dq) for m in low_state.motor_state]
    msg.effort = [float(m.tau_est) for m in low_state.motor_state]
    return msg
