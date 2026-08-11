import os
from typing import Any, Dict

import yaml


def _default_registry_path() -> str:
    source_path = os.path.join(
        os.path.dirname(__file__), "..", "config", "pnd_joint_registry.yaml"
    )
    if os.path.exists(source_path):
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("pnd_ros2_control")
        return os.path.join(share, "config", "pnd_joint_registry.yaml")
    except Exception:
        return source_path


def load_registry(path: str = None) -> Dict[str, Any]:
    if path is None:
        path = _default_registry_path()
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)
