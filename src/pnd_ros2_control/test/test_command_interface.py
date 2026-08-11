from pnd_ros2_control.pnd_command_interface import PndCommandInterface
from pnd_ros2_control.pnd_joint_registry import load_registry


def test_lowcmd_uses_registry_mapping_and_hold_state():
    cmd_iface = PndCommandInterface(load_registry())
    for joint in cmd_iface.body_joints:
        cmd_iface.update_body_state(joint["ros_name"], 0.1, 0.01, 0.5)

    cmd_iface.set_body_command("left_shoulder_pitch_joint", 0.5)
    low_cmd = cmd_iface.build_low_cmd()

    assert len(low_cmd.motor_cmd) == 31
    assert low_cmd.motor_cmd[15].q == 0.5
    assert low_cmd.motor_cmd[15].kp == 18
    assert low_cmd.motor_cmd[12].q == 0.1
    assert low_cmd.motor_cmd[12].dq == 0.0


def test_handcmd_uses_registry_order():
    cmd_iface = PndCommandInterface(load_registry())
    cmd_iface.set_hand_command("left_pinky_joint", 100)
    cmd_iface.set_hand_command("right_thumb_joint", 900)
    hand_cmd = cmd_iface.build_hand_cmd()

    assert hand_cmd.position[0] == 100
    assert hand_cmd.position[10] == 900
    assert len(cmd_iface.hand_joints) == 12


def test_idle_lowcmd_uses_recorded_start_position_and_idle_gains():
    cmd_iface = PndCommandInterface(load_registry())
    cmd_iface.idle_kp = 5.0
    cmd_iface.idle_kd = 1.0
    cmd_iface.record_idle_body_position("left_shoulder_pitch_joint", 0.7)
    cmd_iface.record_idle_body_position("waist_yaw_joint", -0.1)

    low_cmd = cmd_iface.build_low_cmd()

    assert low_cmd.motor_cmd[15].q == 0.7
    assert low_cmd.motor_cmd[15].kp == 5.0
    assert low_cmd.motor_cmd[15].kd == 1.0
    assert low_cmd.motor_cmd[12].q == -0.1
    assert low_cmd.motor_cmd[12].kp == 5.0


def test_idle_handcmd_uses_recorded_start_position():
    cmd_iface = PndCommandInterface(load_registry())
    cmd_iface.record_idle_hand_position("left_pinky_joint", 123.0)
    cmd_iface.record_idle_hand_position("right_thumb_joint", 456.0)

    hand_cmd = cmd_iface.build_hand_cmd()

    assert hand_cmd.position[0] == 123
    assert hand_cmd.position[10] == 456


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
