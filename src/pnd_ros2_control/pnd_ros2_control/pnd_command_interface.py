from typing import Any, Dict, List, Optional

from pndbotics_sdk_py.idl.default import (
    pnd_adam_msg_dds__HandCmd_,
    pnd_adam_msg_dds__LowCmd_,
)

from .pnd_joint_registry import load_registry


class PndCommandInterface:
    def __init__(self, registry: Optional[Dict[str, Any]] = None):
        self.registry = registry or load_registry()
        self.body_joints = self.registry["body"]["joints"]
        self.hand_joints = self.registry["hands"]["joints"]
        self.body_commands: Dict[str, Dict[str, float]] = {}
        self.hand_commands: Dict[str, float] = {}
        self.current_body_state: Dict[str, Dict[str, float]] = {}
        self.current_hand_state: Dict[str, float] = {}
        self.idle_body_positions: Dict[str, float] = {}
        self.idle_hand_positions: Dict[str, float] = {}
        self.idle_kp = 5.0
        self.idle_kd = 1.0

    def set_body_command(
        self,
        ros_name: str,
        position: float,
        velocity: Optional[float] = None,
        effort: Optional[float] = None,
    ):
        self.body_commands[ros_name] = {
            "position": float(position),
            "velocity": 0.0 if velocity is None else float(velocity),
            "effort": 0.0 if effort is None else float(effort),
        }

    def set_hand_command(self, ros_name: str, position: float):
        self.hand_commands[ros_name] = float(position)

    def clear_body_commands(self):
        self.body_commands.clear()

    def clear_hand_commands(self):
        self.hand_commands.clear()

    def has_body_commands(self) -> bool:
        return bool(self.body_commands)

    def has_hand_commands(self) -> bool:
        return bool(self.hand_commands)

    def update_body_state(
        self, ros_name: str, position: float, velocity: float, effort: float
    ):
        self.current_body_state[ros_name] = {
            "position": float(position),
            "velocity": float(velocity),
            "effort": float(effort),
        }

    def update_hand_state(self, ros_name: str, position: float):
        self.current_hand_state[ros_name] = float(position)

    def record_idle_body_position(self, ros_name: str, position: float):
        self.idle_body_positions.setdefault(ros_name, float(position))

    def record_idle_hand_position(self, ros_name: str, position: float):
        self.idle_hand_positions.setdefault(ros_name, float(position))

    def build_low_cmd(self) -> Any:
        cmd = pnd_adam_msg_dds__LowCmd_(len(self.body_joints))
        cmd.mode_pr = 1
        for joint in self.body_joints:
            idx = int(joint["sdk"])
            motor_cmd = cmd.motor_cmd[idx]
            command = self.body_commands.get(joint["ros_name"])
            state = self.current_body_state.get(joint["ros_name"], {})
            motor_cmd.mode = 1
            if command is not None:
                motor_cmd.q = command["position"]
                motor_cmd.dq = command["velocity"]
                motor_cmd.tau = command["effort"]
                motor_cmd.kp = joint["kp"]
                motor_cmd.kd = joint["kd"]
            else:
                motor_cmd.q = self.idle_body_positions.get(
                    joint["ros_name"], state.get("position", 0.0)
                )
                motor_cmd.dq = 0.0
                motor_cmd.tau = 0.0
                motor_cmd.kp = self.idle_kp
                motor_cmd.kd = self.idle_kd
        return cmd

    def build_hand_cmd(self) -> Any:
        cmd = pnd_adam_msg_dds__HandCmd_()
        for idx, joint in enumerate(self.hand_joints):
            ros_name = joint["ros_name"]
            position = self.hand_commands.get(
                ros_name,
                self.idle_hand_positions.get(
                    ros_name,
                    self.current_hand_state.get(ros_name, 0.0),
                ),
            )
            cmd.position[idx] = int(max(0.0, round(float(position))))
        return cmd
