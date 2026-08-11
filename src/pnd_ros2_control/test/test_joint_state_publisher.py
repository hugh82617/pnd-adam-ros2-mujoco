from builtin_interfaces.msg import Time

from pnd_ros2_control.pnd_joint_registry import load_registry
from pnd_ros2_control.pnd_joint_state_publisher import build_joint_state


def test_joint_state_has_43_joints():
    registry = load_registry()
    body = registry["body"]["joints"]
    hands = registry["hands"]["joints"]
    names = [j["ros_name"] for j in body] + [j["ros_name"] for j in hands]

    body_states = {
        j["ros_name"]: {
            "position": 0.1,
            "velocity": 0.01,
            "effort": 1.0,
        }
        for j in body
    }
    hand_states = {j["ros_name"]: 100.0 for j in hands}

    msg = build_joint_state(
        Time(sec=1, nanosec=0),
        names,
        body_states,
        hand_states,
    )

    assert len(msg.name) == 43
    assert msg.name[15] == "left_shoulder_pitch_joint"
    assert msg.position[15] == 0.1
    assert msg.position[31] == 100.0


def _run_all():
    tests = [
        value
        for name, value in sorted(globals().items())
        if name.startswith("test_") and callable(value)
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")


if __name__ == "__main__":
    _run_all()
