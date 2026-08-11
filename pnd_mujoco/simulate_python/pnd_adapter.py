"""PND MuJoCo adapter: mapping, feedback, and watchdog.

This module keeps the DDS/IDL types untouched and translates between the PND
SDK motor order and the MuJoCo actuator order.
"""

import math
import os
import time
from typing import Any, Dict, List, Optional

import mujoco
import numpy as np
import yaml


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
UINT32_MAX = 0xFFFFFFFF


def load_motor_mapping(path: Optional[str] = None) -> Dict[str, Any]:
    path = path or os.path.join(BASE_DIR, "motor_mapping.yaml")
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["sdk_to_mujoco"] = {
        int(k): int(v) for k, v in cfg["sdk_to_mujoco"].items()
    }
    cfg["sdk_motor_names"] = {
        int(k): v for k, v in cfg.get("sdk_motor_names", {}).items()
    }
    return cfg


def load_hand_mapping(path: Optional[str] = None) -> Dict[str, Any]:
    path = path or os.path.join(BASE_DIR, "hand_mapping.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _name2id(model: mujoco.MjModel, obj_type: Any, name: str) -> int:
    return mujoco.mj_name2id(model, obj_type, name)


def quat_to_ypr(q: Any) -> List[float]:
    """Convert a (w,x,y,z) quaternion to yaw-pitch-roll in radians."""
    w, x, y, z = (float(v) for v in q[:4])
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [yaw, pitch, roll]


class PndAdapter:
    def __init__(
        self,
        model: mujoco.MjModel,
        motor_cfg: Dict[str, Any],
        hand_cfg: Dict[str, Any],
        num_motor: int,
    ):
        self.model = model
        self.num_motor = num_motor
        self.sdk_to_ctrl = motor_cfg["sdk_to_mujoco"]
        self.sdk_names = motor_cfg.get("sdk_motor_names", {})

        expected = set(range(num_motor))
        if set(self.sdk_to_ctrl) != expected:
            missing = sorted(expected - set(self.sdk_to_ctrl))
            extra = sorted(set(self.sdk_to_ctrl) - expected)
            raise ValueError(
                f"motor_mapping keys mismatch: missing={missing}, extra={extra}"
            )

        self.motor_ctrl_ids = sorted(
            {self.sdk_to_ctrl[i] for i in range(num_motor)}
        )
        self.ctrl_to_dof = self._build_ctrl_dof()
        self.ctrl_to_qpos = self._build_ctrl_qpos()
        self.hand_groups = self._build_hand_groups(hand_cfg)
        self.hand_ctrl_ids = sorted(
            {ctrl_id for group in self.hand_groups for ctrl_id in group["ctrl"]}
        )
        self.conversion = hand_cfg.get("conversion", {})
        self.watchdog_timeout = (
            hand_cfg.get("watchdog_timeout_ms", 100) / 1000.0
        )

        self.sensor_adr = {}
        for sensor_name in ("imu_quat", "imu_gyro", "imu_acc"):
            sensor_id = _name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name
            )
            if sensor_id >= 0:
                self.sensor_adr[sensor_name] = int(
                    model.sensor_adr[sensor_id]
                )

        self.last_lowcmd_time = 0.0
        self.last_handcmd_time = 0.0
        self.last_kd = np.zeros(model.nu, dtype=float)
        self.last_mode = [0] * num_motor
        self.last_mode_pr = 0
        self.motor_active = False

    def _build_ctrl_dof(self) -> Dict[int, int]:
        result = {}
        for ctrl_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[ctrl_id, 0])
            result[ctrl_id] = int(self.model.jnt_dofadr[joint_id])
        return result

    def _build_ctrl_qpos(self) -> Dict[int, int]:
        result = {}
        for ctrl_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[ctrl_id, 0])
            result[ctrl_id] = int(self.model.jnt_qposadr[joint_id])
        return result

    def _build_hand_groups(
        self, hand_cfg: Dict[str, Any]
    ) -> List[Dict[str, Any]]:
        groups = []
        for group_cfg in hand_cfg.get("hand_position_groups", []):
            group = {
                "name": group_cfg["name"],
                "kind": group_cfg["kind"],
                "qpos": [],
                "ctrl": [],
            }
            for joint_name in group_cfg["joints"]:
                joint_id = _name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
                actuator_id = _name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
                )
                if joint_id >= 0:
                    group["qpos"].append(int(self.model.jnt_qposadr[joint_id]))
                if actuator_id >= 0:
                    group["ctrl"].append(int(actuator_id))
            groups.append(group)
        return groups

    def ApplyLowCmd(self, cmd: Any, data: mujoco.MjData, now: Optional[float] = None):
        if len(cmd.motor_cmd) != self.num_motor:
            raise ValueError(
                f"LowCmd has {len(cmd.motor_cmd)} motors, expected {self.num_motor}"
            )
        self.last_lowcmd_time = time.monotonic() if now is None else now
        self.motor_active = True
        self.last_mode_pr = int(getattr(cmd, "mode_pr", 0))

        for sdk_id in range(self.num_motor):
            ctrl_id = self.sdk_to_ctrl[sdk_id]
            motor_cmd = cmd.motor_cmd[sdk_id]
            q_meas = float(data.sensordata[ctrl_id])
            dq_meas = float(data.sensordata[ctrl_id + self.num_motor])
            data.ctrl[ctrl_id] = (
                motor_cmd.tau
                + motor_cmd.kp * (motor_cmd.q - q_meas)
                + motor_cmd.kd * (motor_cmd.dq - dq_meas)
            )
            self.last_kd[ctrl_id] = float(motor_cmd.kd)
            self.last_mode[sdk_id] = int(motor_cmd.mode)

    def TouchHandCommand(self, now: Optional[float] = None):
        self.last_handcmd_time = time.monotonic() if now is None else now

    def ApplyWatchdog(
        self,
        data: mujoco.MjData,
        now: Optional[float] = None,
        timeout: Optional[float] = None,
    ):
        timeout = self.watchdog_timeout if timeout is None else timeout
        now = time.monotonic() if now is None else now
        if now - self.last_lowcmd_time <= timeout:
            self.motor_active = True
            return

        self.motor_active = False
        for ctrl_id in self.motor_ctrl_ids:
            dof_id = self.ctrl_to_dof[ctrl_id]
            kd = float(self.last_kd[ctrl_id])
            if kd == 0.0:
                kd = float(self.model.dof_damping[dof_id])
            data.ctrl[ctrl_id] = -kd * data.qvel[dof_id]

        for ctrl_id in self.hand_ctrl_ids:
            qpos_id = self.ctrl_to_qpos[ctrl_id]
            data.ctrl[ctrl_id] = data.qpos[qpos_id]

    def FillLowState(
        self,
        state: Any,
        data: mujoco.MjData,
        now: Optional[float] = None,
    ):
        del now
        tick_ms = int(round(float(data.time) * 1000.0)) & UINT32_MAX
        state.tick = tick_ms
        state.mode_pr = self.last_mode_pr if self.motor_active else 0
        state.reserve = 0

        for sdk_id in range(self.num_motor):
            ctrl_id = self.sdk_to_ctrl[sdk_id]
            motor_state = state.motor_state[sdk_id]
            motor_state.q = float(data.sensordata[ctrl_id])
            motor_state.dq = float(
                data.sensordata[ctrl_id + self.num_motor]
            )
            motor_state.ddq = float(
                data.qacc[self.ctrl_to_dof[ctrl_id]]
            )
            motor_state.tau_est = float(
                data.sensordata[ctrl_id + 2 * self.num_motor]
            )
            motor_state.mode = 1 if self.motor_active else 0
            motor_state.state = 1 if self.motor_active else 0
            motor_state.reserve = 0

        self._fill_imu(state, data, tick_ms)
        state.battery_data.timestamp_ms = tick_ms
        state.battery_data.voltage = 0.0
        state.battery_data.current = 0.0
        state.battery_data.power = 0.0
        state.battery_data.wh_accumulated = 0.0
        state.battery_data.status = "SIM_NO_BATTERY_MODEL"

    def _fill_imu(self, state: Any, data: mujoco.MjData, tick_ms: int):
        quat_adr = self.sensor_adr.get("imu_quat")
        if quat_adr is not None:
            q = data.sensordata[quat_adr : quat_adr + 4]
            for i in range(4):
                state.imu_state.quaternion[i] = float(q[i])
            ypr = quat_to_ypr(q)
            for i in range(3):
                state.imu_state.ypr[i] = ypr[i]

        gyro_adr = self.sensor_adr.get("imu_gyro")
        if gyro_adr is not None:
            gyro = data.sensordata[gyro_adr : gyro_adr + 3]
            for i in range(3):
                state.imu_state.gyroscope[i] = float(gyro[i])

        acc_adr = self.sensor_adr.get("imu_acc")
        if acc_adr is not None:
            acc = data.sensordata[acc_adr : acc_adr + 3]
            for i in range(3):
                state.imu_state.accelerometer[i] = float(acc[i])

        state.imu_state.temperature = 0
        state.battery_data.timestamp_ms = tick_ms

    def FillHandState(self, state: Any, data: mujoco.MjData):
        for idx, group in enumerate(self.hand_groups):
            q_values = [float(data.qpos[qpos_id]) for qpos_id in group["qpos"]]
            state.position[idx] = self._joints_to_raw(group["kind"], q_values)
        state.reserve = 0

    def _joints_to_raw(self, kind: str, q_values: List[float]) -> int:
        defaults = {
            "standard": {"q_open": 1.6, "raw_open": 0, "raw_closed": 1000},
            "thumb": {"q_open": 1.0, "raw_open": 0, "raw_closed": 1000},
            "thumb_lateral": {"q_open": 1.0, "raw_open": 0, "raw_closed": 1000},
        }
        cfg = self.conversion.get(kind, defaults[kind])
        q_open = float(cfg.get("q_open", defaults[kind]["q_open"]))
        raw_open = float(cfg.get("raw_open", defaults[kind]["raw_open"]))
        raw_closed = float(cfg.get("raw_closed", defaults[kind]["raw_closed"]))

        if kind == "thumb_lateral":
            q = q_values[-1] if q_values else q_open
        else:
            q = float(np.mean(q_values)) if q_values else q_open

        q_ratio = min(1.0, max(0.0, q / q_open)) if q_open else 1.0
        ratio = 1.0 - q_ratio
        raw = raw_open + (raw_closed - raw_open) * ratio
        return int(round(max(min(raw, 4294967295.0), 0.0)))
