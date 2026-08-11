#!/usr/bin/env python3
"""Generate a static Xacro URDF and joint_limits.yaml from PND Adam Pro MJCF."""

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

import yaml


ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
DEFAULT_SOURCE = os.path.join(
    ROOT, "pnd_mujoco", "pnd_robots", "adam_pro", "adam_pro.xml"
)
DEFAULT_MESH_SOURCE = os.path.join(
    ROOT, "pnd_mujoco", "pnd_robots", "adam_pro", "assets"
)
DEFAULT_OUTPUT_DIR = os.path.join(
    ROOT, "pnd_adam_ros2_publish", "src", "pnd_adam_description"
)


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


def parse_vec(text: Optional[str], default: List[float]) -> List[float]:
    if not text:
        return list(default)
    return [float(v) for v in text.replace(",", " ").split()]


def quat_to_rpy(q: List[float]) -> List[float]:
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [roll, pitch, yaw]


def fmt_vec(values: List[float]) -> str:
    return " ".join(f"{v:.9g}" for v in values)


def link_name(body_name: str) -> str:
    explicit = {
        "pelvis": "base_link",
        "hipPitchLeft": "left_hip_pitch_link",
        "hipRollLeft": "left_hip_roll_link",
        "hipYawLeft": "left_hip_yaw_link",
        "thighLeft": "left_thigh_link",
        "shinLeft": "left_shin_link",
        "anklePitchLeft": "left_ankle_pitch_link",
        "ankleRollLeft": "left_ankle_roll_link",
        "toeLeft": "left_foot_link",
        "hipPitchRight": "right_hip_pitch_link",
        "hipRollRight": "right_hip_roll_link",
        "hipYawRight": "right_hip_yaw_link",
        "thighRight": "right_thigh_link",
        "shinRight": "right_shin_link",
        "anklePitchRight": "right_ankle_pitch_link",
        "ankleRollRight": "right_ankle_roll_link",
        "toeRight": "right_foot_link",
        "waistRoll_link": "waist_roll_link",
        "waistPitch_link": "waist_pitch_link",
        "torso": "torso_link",
        "shoulderPitchLeft": "left_shoulder_pitch_link",
        "shoulderRollLeft": "left_shoulder_roll_link",
        "shoulderYawLeft": "left_shoulder_yaw_link",
        "elbowLeft": "left_elbow_link",
        "wristYawLeft": "left_wrist_yaw_link",
        "wristPitchLeft": "left_wrist_pitch_link",
        "wristRollLeft": "left_wrist_roll_link",
        "shoulderPitchRight": "right_shoulder_pitch_link",
        "shoulderRollRight": "right_shoulder_roll_link",
        "shoulderYawRight": "right_shoulder_yaw_link",
        "elbowRight": "right_elbow_link",
        "wristYawRight": "right_wrist_yaw_link",
        "wristPitchRight": "right_wrist_pitch_link",
        "wristRollRight": "right_wrist_roll_link",
        "neckYaw_link": "head_yaw_link",
        "neckPitch_link": "head_pitch_link",
    }
    if body_name in explicit:
        return explicit[body_name]

    mapped = body_name
    if mapped.startswith("L_"):
        mapped = "left_" + mapped[2:]
    elif mapped.startswith("R_"):
        mapped = "right_" + mapped[2:]
    snake = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", mapped)
    snake = re.sub(r"[^a-zA-Z0-9_]", "_", snake).lower()
    if snake.endswith("_link"):
        return snake
    return f"{snake}_link"


def build_mesh_map(root: ET.Element) -> Dict[str, Tuple[str, List[float]]]:
    result = {}
    asset = root.find("asset")
    if asset is None:
        return result
    for mesh in asset.findall("mesh"):
        name = mesh.get("name")
        if not name:
            continue
        scale = parse_vec(mesh.get("scale"), [1.0, 1.0, 1.0])
        result[name] = (mesh.get("file", ""), scale)
    return result


def add_inertial(link: ET.Element, body: ET.Element):
    inertial = body.find("inertial")
    if inertial is not None:
        pos = parse_vec(inertial.get("pos"), [0.0, 0.0, 0.0])
        quat = parse_vec(inertial.get("quat"), [1.0, 0.0, 0.0, 0.0])
        mass = float(inertial.get("mass", "0.01"))
        diag = parse_vec(
            inertial.get("diaginertia"), [0.0001, 0.0001, 0.0001]
        )
        ixx, iyy, izz = diag[0], diag[1], diag[2]
        el = ET.SubElement(link, "inertial")
        ET.SubElement(
            el,
            "origin",
            {"xyz": fmt_vec(pos), "rpy": fmt_vec(quat_to_rpy(quat))},
        )
        ET.SubElement(
            el,
            "mass",
            {"value": f"{mass:.9g}"},
        )
        ET.SubElement(
            el,
            "inertia",
            {
                "ixx": f"{ixx:.9g}",
                "iyy": f"{iyy:.9g}",
                "izz": f"{izz:.9g}",
                "ixy": "0",
                "ixz": "0",
                "iyz": "0",
            },
        )
        return

    el = ET.SubElement(link, "inertial")
    ET.SubElement(el, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(el, "mass", {"value": "0.01"})
    ET.SubElement(
        el,
        "inertia",
        {
            "ixx": "0.0001",
            "iyy": "0.0001",
            "izz": "0.0001",
            "ixy": "0",
            "ixz": "0",
            "iyz": "0",
        },
    )


def add_visuals(
    link: ET.Element,
    body: ET.Element,
    mesh_map: Dict[str, Tuple[str, List[float]]],
):
    for geom in body.findall("geom"):
        if geom.get("class") == "collision":
            continue
        if geom.get("mesh") is None:
            continue
        mesh_name = geom.get("mesh")
        mesh_info = mesh_map.get(mesh_name or "")
        if mesh_info is None:
            continue
        mesh_file, mesh_scale = mesh_info
        pos = parse_vec(geom.get("pos"), [0.0, 0.0, 0.0])
        quat = parse_vec(geom.get("quat"), [1.0, 0.0, 0.0, 0.0])
        visual = ET.SubElement(link, "visual")
        ET.SubElement(
            visual,
            "origin",
            {"xyz": fmt_vec(pos), "rpy": fmt_vec(quat_to_rpy(quat))},
        )
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(
            geometry,
            "mesh",
            {
                "filename": (
                    "package://pnd_adam_description/meshes/"
                    + os.path.basename(mesh_file)
                ),
                "scale": fmt_vec(mesh_scale),
            },
        )


def walk_body(
    robot: ET.Element,
    body: ET.Element,
    parent_link: Optional[str],
    mesh_map: Dict[str, Tuple[str, List[float]]],
    joint_limits: Dict[str, Dict[str, object]],
):
    body_name = body.get("name", "")
    child_link = link_name(body_name)
    body_pos = parse_vec(body.get("pos"), [0.0, 0.0, 0.0])
    body_quat = parse_vec(body.get("quat"), [1.0, 0.0, 0.0, 0.0])

    link = ET.SubElement(robot, "link", {"name": child_link})
    if child_link != "base_link":
        add_inertial(link, body)
    add_visuals(link, body, mesh_map)

    joint_el = body.find("joint")
    if parent_link is not None and joint_el is not None:
        mjcf_joint_name = joint_el.get("name", "")
        ros_joint_name = JOINT_NAMES.get(mjcf_joint_name)
        joint_type = "revolute" if ros_joint_name else "fixed"
        joint = ET.SubElement(
            robot,
            "joint",
            {
                "name": ros_joint_name or f"fixed_{child_link}",
                "type": joint_type,
            },
        )
        ET.SubElement(joint, "parent", {"link": parent_link})
        ET.SubElement(joint, "child", {"link": child_link})

        joint_pos = parse_vec(joint_el.get("pos"), [0.0, 0.0, 0.0])
        origin_pos = body_pos
        if joint_pos != [0.0, 0.0, 0.0]:
            origin_pos = [body_pos[i] + joint_pos[i] for i in range(3)]
        ET.SubElement(
            joint,
            "origin",
            {
                "xyz": fmt_vec(origin_pos),
                "rpy": fmt_vec(quat_to_rpy(body_quat)),
            },
        )

        if ros_joint_name:
            axis = parse_vec(joint_el.get("axis"), [0.0, 0.0, 1.0])
            ET.SubElement(joint, "axis", {"xyz": fmt_vec(axis)})
            lower, upper = parse_vec(
                joint_el.get("range"), [-3.14, 3.14]
            )[:2]
            effort = float(joint_el.get("actuatorfrcrange", "100 100").split()[1])
            limit = ET.SubElement(joint, "limit")
            limit.set("lower", f"{lower:.9g}")
            limit.set("upper", f"{upper:.9g}")
            limit.set("effort", f"{effort:.9g}")
            limit.set("velocity", "5.0")
            joint_limits[ros_joint_name] = {
                "has_position_limits": True,
                "min_position": lower,
                "max_position": upper,
                "max_velocity": 5.0,
                "max_effort": effort,
            }
        else:
            ET.SubElement(
                joint,
                "origin",
                {
                    "xyz": fmt_vec(origin_pos),
                    "rpy": fmt_vec(quat_to_rpy(body_quat)),
                },
            )
    elif parent_link is not None:
        fixed = ET.SubElement(
            robot,
            "joint",
            {
                "name": f"fixed_{child_link}",
                "type": "fixed",
            },
        )
        ET.SubElement(fixed, "parent", {"link": parent_link})
        ET.SubElement(fixed, "child", {"link": child_link})
        ET.SubElement(
            fixed,
            "origin",
            {
                "xyz": fmt_vec(body_pos),
                "rpy": fmt_vec(quat_to_rpy(body_quat)),
            },
        )

    for child in body.findall("body"):
        walk_body(robot, child, child_link, mesh_map, joint_limits)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    parser.add_argument("--mesh-source", default=DEFAULT_MESH_SOURCE)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    mesh_map = build_mesh_map(root)

    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
    robot = ET.Element(
        "robot",
        {
            "name": "adam_pro",
        },
    )
    ET.SubElement(
        robot,
        "{http://www.ros.org/wiki/xacro}property",
        {"name": "mesh_dir", "value": "package://pnd_adam_description/meshes/"},
    )

    joint_limits: Dict[str, Dict[str, object]] = {}
    worldbody = root.find("worldbody")
    if worldbody is not None:
        for body in worldbody.findall("body"):
            walk_body(robot, body, None, mesh_map, joint_limits)

    urdf_dir = os.path.join(args.output_dir, "urdf")
    config_dir = os.path.join(args.output_dir, "config")
    os.makedirs(urdf_dir, exist_ok=True)
    os.makedirs(config_dir, exist_ok=True)

    ET.indent(robot, space="  ")
    urdf_path = os.path.join(urdf_dir, "adam_pro.urdf.xacro")
    tree_out = ET.ElementTree(robot)
    tree_out.write(urdf_path, encoding="utf-8", xml_declaration=True)

    limits_path = os.path.join(config_dir, "joint_limits.yaml")
    with open(limits_path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"joint_limits": joint_limits}, f, sort_keys=False)

    print(f"Wrote {urdf_path}")
    print(f"Wrote {limits_path}")


if __name__ == "__main__":
    main()
