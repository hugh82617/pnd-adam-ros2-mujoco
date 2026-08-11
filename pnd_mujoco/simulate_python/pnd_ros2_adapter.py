import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState

from pnd_adapter import load_hand_mapping, load_motor_mapping
from ros2_adapter.hand_state_publisher import build_hand_state
from ros2_adapter.imu_publisher import build_imu_msg
from ros2_adapter.joint_state_publisher import build_joint_state


class PndRos2Adapter(Node):

    def __init__(self):

        super().__init__(
            "pnd_adam_ros2_bridge"
        )

        motor_cfg = load_motor_mapping()
        hand_cfg = load_hand_mapping()

        self.joint_names = [
            motor_cfg["ros_joint_names"][i]
            for i in sorted(motor_cfg["sdk_to_mujoco"])
        ]
        self.hand_names = [
            group["name"]
            for group in hand_cfg["hand_position_groups"]
        ]

        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/data",
            10
        )

        self.hand_pub = self.create_publisher(
            JointState,
            "/hand_state",
            10
        )

    def publish_low_state(self, low_state):
        if not rclpy.ok():
            return
        stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(
            build_joint_state(stamp, self.joint_names, low_state)
        )
        self.imu_pub.publish(build_imu_msg(stamp, low_state))

    def publish_hand_state(self, hand_state):
        if not rclpy.ok():
            return
        stamp = self.get_clock().now().to_msg()
        self.hand_pub.publish(
            build_hand_state(stamp, self.hand_names, hand_state)
        )
