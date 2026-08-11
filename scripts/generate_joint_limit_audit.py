#!/usr/bin/env python3
"""Generate JOINT_LIMIT_AUDIT.md from MJCF, URDF and MoveIt limits."""

import os
import xml.etree.ElementTree as ET

import yaml


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF = os.path.join(
    ROOT, "pnd_mujoco", "pnd_robots", "adam_pro", "adam_pro.xml"
)
URDF = os.path.join(ROOT, "src", "pnd_adam_description", "urdf", "pnd_adam.urdf")
MOVEIT_LIMITS = os.path.join(
    ROOT, "src", "pnd_adam_moveit_config", "config", "joint_limits.yaml"
)
OUTPUT = os.path.join(ROOT, "JOINT_LIMIT_AUDIT.md")


JOINT_NAMES = {
    "hipPitch_Left": "left_hip_pitch_joint",
    "hipRoll_Left": "left_hip_roll_joint",
    "hipYaw_Left": "left_hip_yaw_joint",
    "kneePitch_Left": "left_knee_pitch_joint",
    "anklePitch_Left": "left_ankle_pitch_joint",
    "ankleRoll_Left": "left_ankle_roll_joint",
    "hipPitch_Right": "right_hip_pitch_joint",
    "hipRoll_Right": "right_hip_roll_joint",
    "hipYaw_Right": "right_hip_yaw_joint",
    "kneePitch_Right": "right_knee_pitch_joint",
    "anklePitch_Right": "right_ankle_pitch_joint",
    "ankleRoll_Right": "right_ankle_roll_joint",
    "waistYaw": "waist_yaw_joint",
    "waistRoll": "waist_roll_joint",
    "waistPitch": "waist_pitch_joint",
    "shoulderPitch_Left": "left_shoulder_pitch_joint",
    "shoulderRoll_Left": "left_shoulder_roll_joint",
    "shoulderYaw_Left": "left_shoulder_yaw_joint",
    "elbow_Left": "left_elbow_joint",
    "wristYaw_Left": "left_wrist_yaw_joint",
    "wristPitch_Left": "left_wrist_pitch_joint",
    "wristRoll_Left": "left_wrist_roll_joint",
    "shoulderPitch_Right": "right_shoulder_pitch_joint",
    "shoulderRoll_Right": "right_shoulder_roll_joint",
    "shoulderYaw_Right": "right_shoulder_yaw_joint",
    "elbow_Right": "right_elbow_joint",
    "wristYaw_Right": "right_wrist_yaw_joint",
    "wristPitch_Right": "right_wrist_pitch_joint",
    "wristRoll_Right": "right_wrist_roll_joint",
    "neckYaw": "head_yaw_joint",
    "neckPitch": "head_pitch_joint",
}


def parse_mjcf(path):
    root = ET.parse(path).getroot()
    limits = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        ros_name = JOINT_NAMES.get(name)
        if not ros_name or not joint.get("range"):
            continue
        lower, upper = [float(v) for v in joint.get("range").split()]
        limits[ros_name] = (lower, upper)
    return limits


def parse_urdf(path):
    root = ET.parse(path).getroot()
    limits = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        if name not in JOINT_NAMES.values():
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits[name] = (
            float(limit.get("lower")),
            float(limit.get("upper")),
        )
    return limits


def parse_moveit_limits(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    limits = {}
    for name, entry in cfg.get("joint_limits", {}).items():
        if name not in JOINT_NAMES.values():
            continue
        limits[name] = (entry.get("min_position"), entry.get("max_position"))
    return limits


def fmt(value):
    if value is None:
        return "N/A"
    return f"{value:g}"


def main():
    mjcf = parse_mjcf(MJCF)
    urdf = parse_urdf(URDF)
    moveit = parse_moveit_limits(MOVEIT_LIMITS)

    rows = []
    conflicts = []
    for name in sorted(JOINT_NAMES.values()):
        m = mjcf.get(name, (None, None))
        u = urdf.get(name, (None, None))
        mi = moveit.get(name, (None, None))
        initial = 0.0
        conflict = None
        if m[0] is not None and initial < m[0]:
            conflict = f"initial {initial:g} < MuJoCo min {fmt(m[0])}"
        if m[1] is not None and initial > m[1]:
            conflict = f"initial {initial:g} > MuJoCo max {fmt(m[1])}"
        if mi[0] is not None and initial < mi[0]:
            conflict = f"initial {initial:g} < MoveIt min {fmt(mi[0])}"
        if mi[1] is not None and initial > mi[1]:
            conflict = f"initial {initial:g} > MoveIt max {fmt(mi[1])}"
        if conflict:
            conflicts.append((name, conflict))
        rows.append(
            (
                name,
                f"{fmt(m[0])} .. {fmt(m[1])}",
                f"{fmt(u[0])} .. {fmt(u[1])}",
                f"{fmt(mi[0])} .. {fmt(mi[1])}",
                fmt(initial),
                conflict or "ok",
            )
        )

    lines = [
        "# JOINT_LIMIT_AUDIT",
        "",
        "来源：",
        "",
        "- MuJoCo: `pnd_mujoco/pnd_robots/adam_pro/adam_pro.xml`",
        "- URDF: `src/pnd_adam_description/urdf/pnd_adam.urdf`",
        "- MoveIt: `src/pnd_adam_moveit_config/config/joint_limits.yaml`",
        "- 官方初始 body pose: `0`",
        "",
        "| Joint | MuJoCo range | URDF range | MoveIt range | Initial | Conflict |",
        "|---|---|---|---|---|---|",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")

    if conflicts:
        lines.extend(
            [
                "",
                "## 冲突",
                "",
            ]
        )
        for name, message in conflicts:
            lines.append(f"- `{name}`: {message}")
    else:
        lines.extend(["", "## 冲突", "", "未发现初始 pose 超出范围的问题。"])

    with open(OUTPUT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {OUTPUT}")


if __name__ == "__main__":
    main()
