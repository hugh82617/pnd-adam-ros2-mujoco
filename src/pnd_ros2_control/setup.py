from setuptools import find_packages, setup

package_name = "pnd_ros2_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/pnd_joint_registry.yaml"]),
        ("share/" + package_name + "/launch", ["launch/pnd_control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PND Digital Twin",
    maintainer_email="dev@pndbotics.local",
    description="Python ROS2 control bridge for PND Adam Pro.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "pnd_ros2_control_bridge = pnd_ros2_control.pnd_ros2_control_bridge:main",
            "pnd_joint_state_publisher = pnd_ros2_control.pnd_joint_state_publisher:main",
            "pnd_arm_swing_demo = pnd_ros2_control.pnd_arm_swing_demo:main",
        ],
    },
)
