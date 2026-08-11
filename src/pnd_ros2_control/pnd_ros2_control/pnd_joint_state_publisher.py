import threading
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pndbotics_sdk_py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import (
    HandState_,
    LowState_,
)

from .pnd_joint_registry import load_registry


def build_joint_state(
    stamp: Any,
    joint_names: List[str],
    body_states: Dict[str, Dict[str, float]],
    hand_states: Dict[str, float],
) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.name = list(joint_names)
    msg.position = []
    msg.velocity = []
    msg.effort = []
    for name in joint_names:
        if name in body_states:
            state = body_states[name]
            msg.position.append(state.get("position", 0.0))
            msg.velocity.append(state.get("velocity", 0.0))
            msg.effort.append(state.get("effort", 0.0))
        else:
            msg.position.append(hand_states.get(name, 0.0))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
    return msg


class PndJointStatePublisher(Node):
    def __init__(self):
        super().__init__("pnd_joint_state_publisher")
        self.declare_parameter("domain_id", 1)
        self.declare_parameter("interface", "lo")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("publish_hand_joints", True)

        domain_id = int(self.get_parameter("domain_id").value)
        interface = str(self.get_parameter("interface").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        hand_joints_value = self.get_parameter("publish_hand_joints").value
        if isinstance(hand_joints_value, bool):
            publish_hand_joints = hand_joints_value
        else:
            publish_hand_joints = str(hand_joints_value).lower() in (
                "1",
                "true",
                "yes",
                "on",
            )

        registry = load_registry()
        self.body_joints = registry["body"]["joints"]
        self.hand_joints = registry["hands"]["joints"]
        self.joint_names = [joint["ros_name"] for joint in self.body_joints]
        if publish_hand_joints:
            self.joint_names += [
                joint["ros_name"] for joint in self.hand_joints
            ]

        self.lock = threading.Lock()
        self.body_states: Dict[str, Dict[str, float]] = {}
        self.hand_states: Dict[str, float] = {}

        ChannelFactoryInitialize(domain_id, interface)
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self.on_lowstate, 1)
        self.handstate_sub = ChannelSubscriber("rt/handstate", HandState_)
        self.handstate_sub.Init(self.on_handstate, 1)

        self.publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.create_timer(1.0 / publish_rate, self.tick)

    def on_lowstate(self, msg):
        with self.lock:
            for idx, joint in enumerate(self.body_joints):
                motor = msg.motor_state[idx]
                self.body_states[joint["ros_name"]] = {
                    "position": float(motor.q),
                    "velocity": float(motor.dq),
                    "effort": float(motor.tau_est),
                }

    def on_handstate(self, msg):
        with self.lock:
            for idx, joint in enumerate(self.hand_joints):
                self.hand_states[joint["ros_name"]] = float(
                    msg.position[idx]
                )

    def tick(self):
        with self.lock:
            msg = build_joint_state(
                self.get_clock().now().to_msg(),
                self.joint_names,
                self.body_states,
                self.hand_states,
            )
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PndJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
