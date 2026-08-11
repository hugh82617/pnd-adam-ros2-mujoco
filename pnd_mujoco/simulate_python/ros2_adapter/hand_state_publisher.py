"""Build standard sensor_msgs/JointState from PND HandState."""

from typing import Any, List

from sensor_msgs.msg import JointState


def build_hand_state(
    stamp: Any,
    hand_names: List[str],
    hand_state: Any,
) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = "palm"
    msg.name = list(hand_names)
    msg.position = [float(v) for v in hand_state.position]
    msg.velocity = [0.0] * len(hand_names)
    msg.effort = [0.0] * len(hand_names)
    return msg
