#!/usr/bin/env python3
"""Generate a complete hand self-collision matrix for PND Adam Pro."""

import os
import xml.etree.ElementTree as ET
from itertools import combinations


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(PACKAGE_ROOT, "urdf", "pnd_adam.urdf")
SRDF_FILES = [
    os.path.join(PACKAGE_ROOT, "srdf", "pnd_adam.srdf"),
    os.path.join(
        PACKAGE_ROOT,
        "..",
        "pnd_adam_moveit_config",
        "srdf",
        "pnd_adam.srdf",
    ),
]
HAND_ROOTS = (
    "left_wrist_roll_link",
    "right_wrist_roll_link",
)


def adjacent_pairs(urdf_root: ET.Element) -> set:
    pairs = set()
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        pair = tuple(sorted((parent.get("link"), child.get("link"))))
        pairs.add(pair)
    return pairs


def subtree_links(urdf_root: ET.Element, root_link: str) -> set:
    links = {root_link}
    changed = True
    while changed:
        changed = False
        for joint in urdf_root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            if parent.get("link") in links and child.get("link") not in links:
                links.add(child.get("link"))
                changed = True
    return links


def main():
    urdf_root = ET.parse(URDF).getroot()
    pairs = adjacent_pairs(urdf_root)

    for hand_root in HAND_ROOTS:
        hand_links = subtree_links(urdf_root, hand_root)
        pairs.update(combinations(sorted(hand_links), 2))

    for srdf_path in SRDF_FILES:
        tree = ET.parse(srdf_path)
        srdf_root = tree.getroot()
        for existing in srdf_root.findall("disable_collisions"):
            srdf_root.remove(existing)
        for link1, link2 in sorted(pairs):
            ET.SubElement(
                srdf_root,
                "disable_collisions",
                {"link1": link1, "link2": link2},
            )
        ET.indent(srdf_root, space="  ")
        tree.write(srdf_path, encoding="utf-8", xml_declaration=True)
        print(f"Wrote {srdf_path} ({len(pairs)} disable_collisions)")


if __name__ == "__main__":
    main()
