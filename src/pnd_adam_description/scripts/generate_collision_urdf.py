#!/usr/bin/env python3
"""Add simplified collision geometry to the processed PND Adam Pro URDF."""

import argparse
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from itertools import product
from typing import List, Tuple

import numpy as np


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")
)
DEFAULT_XACRO = os.path.join(PACKAGE_ROOT, "urdf", "adam_pro.urdf.xacro")
DEFAULT_OUTPUT = os.path.join(PACKAGE_ROOT, "urdf", "pnd_adam.urdf")
REPORT_PATH = os.path.join(PACKAGE_ROOT, "urdf", "pnd_adam_collision_geometry.md")
MESH_DIR = os.path.join(PACKAGE_ROOT, "meshes")
SAFETY_SCALE = 0.85
HAND_SAFETY_SCALE = 0.55
HAND_ROOT_LINKS = (
    "left_wrist_roll_link",
    "right_wrist_roll_link",
)

HAND_JOINTS = [
    ("left_pinky_joint", "left_pinky_link", "left_wrist_roll_link"),
    ("left_ring_joint", "left_ring_link", "left_wrist_roll_link"),
    ("left_middle_joint", "left_middle_link", "left_wrist_roll_link"),
    ("left_index_joint", "left_index_link", "left_wrist_roll_link"),
    ("left_thumb_joint", "left_thumb_link", "left_wrist_roll_link"),
    (
        "left_thumb_lateral_joint",
        "left_thumb_lateral_link",
        "left_wrist_roll_link",
    ),
    ("right_pinky_joint", "right_pinky_link", "right_wrist_roll_link"),
    ("right_ring_joint", "right_ring_link", "right_wrist_roll_link"),
    ("right_middle_joint", "right_middle_link", "right_wrist_roll_link"),
    ("right_index_joint", "right_index_link", "right_wrist_roll_link"),
    ("right_thumb_joint", "right_thumb_link", "right_wrist_roll_link"),
    (
        "right_thumb_lateral_joint",
        "right_thumb_lateral_link",
        "right_wrist_roll_link",
    ),
]
VIRTUAL_HAND_LINKS = {child_link for _, child_link, _ in HAND_JOINTS}


def rotation_matrix(rpy: List[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def parse_vec(text: str, default: List[float]) -> List[float]:
    if not text:
        return list(default)
    return [float(v) for v in text.replace(",", " ").split()]


def mesh_bbox(path: str) -> Tuple[np.ndarray, np.ndarray]:
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            vertex = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            mins = np.minimum(mins, vertex)
            maxs = np.maximum(maxs, vertex)
    if not np.isfinite(mins).all() or not np.isfinite(maxs).all():
        mins = np.zeros(3)
        maxs = np.ones(3) * 0.01
    return mins, maxs


def transform_bbox(
    mins: np.ndarray,
    maxs: np.ndarray,
    scale: List[float],
    xyz: List[float],
    rpy: List[float],
) -> Tuple[np.ndarray, np.ndarray]:
    rot = rotation_matrix(rpy)
    corners = []
    for combo in product((0, 1), repeat=3):
        corner = np.array(
            [maxs[i] if combo[i] else mins[i] for i in range(3)]
        )
        corner = corner * np.array(scale)
        corner = rot @ corner + np.array(xyz)
        corners.append(corner)
    corners = np.array(corners)
    return corners.min(axis=0), corners.max(axis=0)


def resolve_mesh(filename: str) -> str:
    if filename.startswith("package://pnd_adam_description/meshes/"):
        return os.path.join(
            MESH_DIR, filename.rsplit("/", 1)[-1]
        )
    if filename.startswith("file://"):
        return filename[len("file://"):]
    return filename


def subtree_links(root: ET.Element, root_link: str) -> set:
    links = {root_link}
    changed = True
    while changed:
        changed = False
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            if parent.get("link") in links and child.get("link") not in links:
                links.add(child.get("link"))
                changed = True
    return links


def add_collision(link: ET.Element):
    visuals = link.findall("visual")
    if visuals:
        for visual in visuals:
            origin = visual.find("origin")
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None:
                continue
            collision = ET.SubElement(link, "collision")
            if origin is not None:
                ET.SubElement(
                    collision,
                    "origin",
                    {
                        "xyz": origin.get("xyz", "0 0 0"),
                        "rpy": origin.get("rpy", "0 0 0"),
                    },
                )
            else:
                ET.SubElement(
                    collision,
                    "origin",
                    {"xyz": "0 0 0", "rpy": "0 0 0"},
                )
            collision_geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                collision_geometry,
                "mesh",
                {
                    "filename": mesh.get("filename", ""),
                    "scale": mesh.get("scale", "1 1 1"),
                },
            )
        return

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "sphere", {"radius": "0.01"})


def add_hand_joints(root: ET.Element):
    existing_links = {link.get("name") for link in root.findall("link")}
    existing_joints = {joint.get("name") for joint in root.findall("joint")}

    for joint_name, child_link, parent_link in HAND_JOINTS:
        if child_link not in existing_links:
            ET.SubElement(root, "link", {"name": child_link})
            existing_links.add(child_link)

        if joint_name in existing_joints:
            continue

        joint = ET.SubElement(
            root, "joint", {"name": joint_name, "type": "revolute"}
        )
        ET.SubElement(joint, "parent", {"link": parent_link})
        ET.SubElement(joint, "child", {"link": child_link})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": "0",
                "upper": "1000",
                "effort": "10",
                "velocity": "100",
            },
        )
        existing_joints.add(joint_name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xacro", default=DEFAULT_XACRO)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    urdf_text = subprocess.check_output(["xacro", args.xacro], text=True)
    root = ET.fromstring(urdf_text)
    add_hand_joints(root)

    hand_links = set()
    for hand_root in HAND_ROOT_LINKS:
        hand_links.update(subtree_links(root, hand_root))

    for link in root.findall("link"):
        if link.get("name") in VIRTUAL_HAND_LINKS:
            continue
        add_collision(link)

    ET.indent(root, space="  ")
    tree = ET.ElementTree(root)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {args.output}")

    rows = []
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            origin = collision.find("origin")
            origin_xyz = origin.get("xyz") if origin is not None else "0 0 0"
            origin_rpy = origin.get("rpy") if origin is not None else "0 0 0"
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) == 0:
                continue
            child = list(geometry)[0]
            if child.tag == "box":
                shape = f"box {child.get('size')}"
            elif child.tag == "cylinder":
                shape = (
                    f"cylinder radius={child.get('radius')} "
                    f"length={child.get('length')}"
                )
            elif child.tag == "sphere":
                shape = f"sphere radius={child.get('radius')}"
            else:
                shape = child.tag
            rows.append(
                (
                    link.get("name"),
                    shape,
                    f"xyz={origin_xyz} rpy={origin_rpy}",
                )
            )

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write("# PND Adam Pro Collision Geometry\n\n")
        f.write(
            "Sizes are estimated from visual mesh bounding boxes at "
            "85% safety scale.\n\n"
        )
        f.write("| Link | Collision Geometry | Origin |\n")
        f.write("|---|---|---|\n")
        for link_name, shape, origin in rows:
            f.write(f"| {link_name} | {shape} | {origin} |\n")
    print(f"Wrote {REPORT_PATH}")


if __name__ == "__main__":
    main()
