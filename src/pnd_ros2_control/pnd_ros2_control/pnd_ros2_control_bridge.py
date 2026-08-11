import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from pndbotics_sdk_py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from pndbotics_sdk_py.idl.default import (
    pnd_adam_msg_dds__HandCmd_,
    pnd_adam_msg_dds__LowCmd_,
)
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import (
    HandCmd_,
    HandState_,
    LowCmd_,
    LowState_,
)

from .pnd_command_interface import PndCommandInterface
from .pnd_joint_registry import load_registry


class PndRos2ControlBridge(Node):
    def __init__(self):
        super().__init__("pnd_ros2_control_bridge")
        self.declare_parameter("domain_id", 1)
        self.declare_parameter("interface", "lo")
        self.declare_parameter("control_rate", 500.0)
        self.declare_parameter("idle_kp", 5.0)
        self.declare_parameter("idle_kd", 1.0)

        domain_id = int(self.get_parameter("domain_id").value)
        interface = str(self.get_parameter("interface").value)
        control_rate = float(self.get_parameter("control_rate").value)
        idle_kp = float(self.get_parameter("idle_kp").value)
        idle_kd = float(self.get_parameter("idle_kd").value)

        self.lock = threading.Lock()
        self.command_interface = PndCommandInterface(load_registry())
        self.command_interface.idle_kp = idle_kp
        self.command_interface.idle_kd = idle_kd
        for joint in self.command_interface.body_joints:
            self.command_interface.record_idle_body_position(
                joint["ros_name"], 0.0
            )
        for joint in self.command_interface.hand_joints:
            self.command_interface.record_idle_hand_position(
                joint["ros_name"], 0.0
            )
        self.trajectory_msg = None
        self.trajectory_received_time = 0.0
        self.state_received = False
        self.hand_state_received = False
        self._state_logged = False
        self._idle_hold_logged = False
        self.last_lowstate_time = 0.0
        self.dds_write_ok = True
        self.active_trajectory_goal = None
        self._action_servers = []
        self._action_callback_group = ReentrantCallbackGroup()

        ChannelFactoryInitialize(domain_id, interface)

        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.handcmd_pub = ChannelPublisher("rt/handcmd", HandCmd_)
        self.handcmd_pub.Init()

        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self.on_lowstate, 1)
        self.handstate_sub = ChannelSubscriber("rt/handstate", HandState_)
        self.handstate_sub.Init(self.on_handstate, 1)

        self.create_subscription(
            JointTrajectory,
            "/body_controller/joint_trajectory",
            self.on_trajectory,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/hand_controller/commands",
            self.on_hand_commands,
            10,
        )

        for action_name in (
            "body_controller/follow_joint_trajectory",
            "body_joint_trajectory_controller/follow_joint_trajectory",
            "left_arm_controller/follow_joint_trajectory",
            "right_arm_controller/follow_joint_trajectory",
            "left_hand_controller/follow_joint_trajectory",
            "right_hand_controller/follow_joint_trajectory",
        ):
            self._action_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    self.execute_trajectory,
                    callback_group=self._action_callback_group,
                    goal_callback=self.on_action_goal,
                    cancel_callback=self.on_action_cancel,
                )
            )

        period = 1.0 / control_rate
        self.create_timer(period, self.control_tick)

    def on_lowstate(self, msg):
        with self.lock:
            for idx, joint in enumerate(self.command_interface.body_joints):
                motor = msg.motor_state[idx]
                self.command_interface.update_body_state(
                    joint["ros_name"],
                    motor.q,
                    motor.dq,
                    motor.tau_est,
                )
                self.command_interface.record_idle_body_position(
                    joint["ros_name"], motor.q
                )
            self.state_received = True
            self.last_lowstate_time = time.monotonic()
            if not self._state_logged:
                self.get_logger().info(
                    "LowState received; control bridge is active"
                )
                self._state_logged = True

    def on_handstate(self, msg):
        with self.lock:
            for idx, joint in enumerate(self.command_interface.hand_joints):
                position = msg.position[idx]
                self.command_interface.update_hand_state(
                    joint["ros_name"], position
                )
                self.command_interface.record_idle_hand_position(
                    joint["ros_name"], position
                )
            self.hand_state_received = True

    def on_trajectory(self, msg: JointTrajectory):
        with self.lock:
            self.trajectory_msg = msg
            self.trajectory_received_time = time.monotonic()
        self.get_logger().info(
            f"Trajectory command received for joints: {msg.joint_names}"
        )

    def on_hand_commands(self, msg: Float64MultiArray):
        with self.lock:
            for idx, joint in enumerate(self.command_interface.hand_joints):
                if idx < len(msg.data):
                    self.command_interface.set_hand_command(
                        joint["ros_name"], msg.data[idx]
                    )
        self.get_logger().info(
            f"Hand command received: {[round(float(v), 2) for v in msg.data[:12]]}"
        )

    def on_action_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def on_action_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        if not trajectory.points:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Trajectory contains no points"
            goal_handle.abort()
            return result

        with self.lock:
            self.trajectory_msg = trajectory
            self.trajectory_received_time = time.monotonic()
            self.active_trajectory_goal = goal_handle

        duration = self._trajectory_duration(trajectory)
        self.get_logger().info(
            f"FollowJointTrajectory accepted: {trajectory.joint_names}, "
            f"duration={duration:.3f}s"
        )

        start_time = time.monotonic()
        feedback_time = 0.0
        while goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                with self.lock:
                    if self.active_trajectory_goal is goal_handle:
                        self.active_trajectory_goal = None
                        self.trajectory_msg = None
                        self.command_interface.clear_body_commands()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Goal canceled by client"
                goal_handle.canceled()
                self.get_logger().info("FollowJointTrajectory canceled")
                return result

            now = time.monotonic()
            lowstate_age = (
                now - self.last_lowstate_time
                if self.state_received
                else float("inf")
            )
            if lowstate_age > 0.5 or not self.dds_write_ok:
                result.error_code = (
                    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                )
                result.error_string = (
                    "DDS/LowState timeout or DDS write failed"
                )
                with self.lock:
                    if self.active_trajectory_goal is goal_handle:
                        self.active_trajectory_goal = None
                        self.trajectory_msg = None
                        self.command_interface.clear_body_commands()
                goal_handle.abort()
                self.get_logger().warn(
                    "FollowJointTrajectory aborted: DDS/LowState timeout"
                )
                return result

            if now - start_time >= duration + 0.5:
                with self.lock:
                    if self.active_trajectory_goal is goal_handle:
                        self.active_trajectory_goal = None
                        self.trajectory_msg = None
                        self.command_interface.clear_body_commands()
                goal_handle.succeed()
                self.get_logger().info("FollowJointTrajectory succeeded")
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.SUCCESSFUL
                )

            if now - feedback_time >= 0.1:
                self._publish_action_feedback(goal_handle, trajectory)
                feedback_time = now
            time.sleep(0.05)

        return result

    @staticmethod
    def _trajectory_duration(msg: JointTrajectory) -> float:
        duration = 0.0
        for point in msg.points:
            point_duration = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )
            duration = max(duration, point_duration)
        return duration

    def _publish_action_feedback(
        self, goal_handle, trajectory: JointTrajectory
    ):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)
        desired = trajectory.points[-1]
        feedback.desired.positions = list(desired.positions)
        feedback.desired.velocities = list(desired.velocities)

        with self.lock:
            actual_positions = [
                self.command_interface.current_hand_state.get(name, 0.0)
                if name
                in {
                    joint["ros_name"]
                    for joint in self.command_interface.hand_joints
                }
                else self.command_interface.current_body_state.get(
                    name, {}
                ).get("position", 0.0)
                for name in trajectory.joint_names
            ]

        feedback.actual.positions = actual_positions
        error_positions = []
        for idx, desired_position in enumerate(feedback.desired.positions):
            actual_position = (
                actual_positions[idx]
                if idx < len(actual_positions)
                else 0.0
            )
            error_positions.append(desired_position - actual_position)
        feedback.error.positions = error_positions

        try:
            goal_handle.publish_feedback(feedback)
        except Exception:
            pass

    def control_tick(self):
        with self.lock:
            if not self.state_received:
                return
            self._apply_trajectory_locked()
            low_cmd = self.command_interface.build_low_cmd()
            hand_cmd = (
                self.command_interface.build_hand_cmd()
                if self.hand_state_received
                else None
            )
            if not self._idle_hold_logged:
                self.get_logger().info(
                    "Idle hold active: kp="
                    f"{self.command_interface.idle_kp}, "
                    f"kd={self.command_interface.idle_kd}"
                )
                self._idle_hold_logged = True

        try:
            self.lowcmd_pub.Write(low_cmd)
            if hand_cmd is not None:
                self.handcmd_pub.Write(hand_cmd)
            self.dds_write_ok = True
        except Exception as exc:
            self.get_logger().warn(f"DDS write failed: {exc}")
            self.dds_write_ok = False

    def _apply_trajectory_locked(self):
        msg = self.trajectory_msg
        if msg is None or not msg.points:
            return
        if len(msg.points) == 1:
            self._apply_point(msg, 0)
            return

        times = [
            p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            for p in msg.points
        ]
        elapsed = time.monotonic() - self.trajectory_received_time
        if elapsed <= times[0]:
            self._apply_point(msg, 0)
            return
        if elapsed >= times[-1]:
            self._apply_point(msg, -1)
            return
        for i in range(len(times) - 1):
            if times[i] <= elapsed <= times[i + 1]:
                duration = max(times[i + 1] - times[i], 1e-6)
                ratio = (elapsed - times[i]) / duration
                self._interpolate_points(msg, i, i + 1, ratio)
                return

    def _apply_point(self, msg: JointTrajectory, point_idx: int):
        point = msg.points[point_idx]
        for idx, ros_name in enumerate(msg.joint_names):
            hand_names = {
                joint["ros_name"] for joint in self.command_interface.hand_joints
            }
            position = (
                point.positions[idx]
                if idx < len(point.positions)
                else 0.0
            )
            if ros_name in hand_names:
                self.command_interface.set_hand_command(ros_name, position)
                continue
            velocity = None
            effort = None
            if idx < len(point.velocities):
                velocity = point.velocities[idx]
            if idx < len(point.effort):
                effort = point.effort[idx]
            self.command_interface.set_body_command(
                ros_name, position, velocity, effort
            )

    def _interpolate_points(
        self, msg: JointTrajectory, a_idx: int, b_idx: int, ratio: float
    ):
        a = msg.points[a_idx]
        b = msg.points[b_idx]
        for idx, ros_name in enumerate(msg.joint_names):
            if idx >= len(a.positions) or idx >= len(b.positions):
                continue
            position = a.positions[idx] + ratio * (
                b.positions[idx] - a.positions[idx]
            )
            hand_names = {
                joint["ros_name"] for joint in self.command_interface.hand_joints
            }
            if ros_name in hand_names:
                self.command_interface.set_hand_command(ros_name, position)
            else:
                self.command_interface.set_body_command(ros_name, position)


def main(args=None):
    rclpy.init(args=args)
    node = PndRos2ControlBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
