import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PndArmSwingDemo(Node):
    def __init__(self):
        super().__init__("pnd_arm_swing_demo")
        self.declare_parameter("joint_name", "left_shoulder_pitch_joint")
        self.declare_parameter("frequency", 0.5)
        self.declare_parameter("amplitude", 0.2)
        self.declare_parameter("baseline", 1.0)

        self.joint_name = str(self.get_parameter("joint_name").value)
        self.frequency = float(self.get_parameter("frequency").value)
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.baseline = float(self.get_parameter("baseline").value)
        self.elapsed = 0.0

        self.publisher = self.create_publisher(
            JointTrajectory, "/body_controller/joint_trajectory", 10
        )
        self.create_timer(0.05, self.tick)

    def tick(self):
        self.elapsed += 0.05
        position = self.baseline + self.amplitude * math.sin(
            2.0 * math.pi * self.frequency * self.elapsed
        )
        velocity = (
            self.amplitude
            * 2.0
            * math.pi
            * self.frequency
            * math.cos(2.0 * math.pi * self.frequency * self.elapsed)
        )

        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [velocity]
        point.time_from_start = Duration(sec=0, nanosec=0)
        msg.points = [point]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PndArmSwingDemo()
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
