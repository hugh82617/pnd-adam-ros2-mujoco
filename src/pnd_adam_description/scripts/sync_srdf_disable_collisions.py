#!/usr/bin/env python3
"""Add adjacent-link collision exclusions to PND Adam SRDF copies."""

import os
import xml.etree.ElementTree as ET


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(
    PACKAGE_ROOT,
    "urdf",
    "pnd_adam.urdf",
)
SRDF_FILES = [
    os.path.join(
        PACKAGE_ROOT,
        "srdf",
        "pnd_adam.srdf",
    ),
    os.path.join(
        PACKAGE_ROOT,
        "..",
        "pnd_adam_moveit_config",
        "srdf",
        "pnd_adam.srdf",
    ),
]


def main():
    urdf_root = ET.parse(URDF).getroot()
    pairs = set()
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        pair = tuple(
            sorted((parent.get("link"), child.get("link")))
        )
        pairs.add(pair)

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
