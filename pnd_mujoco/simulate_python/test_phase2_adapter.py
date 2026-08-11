import config
import mujoco

from pnd_adapter import (
    PndAdapter,
    load_hand_mapping,
    load_motor_mapping,
)
from pndbotics_sdk_py.idl.default import (
    pnd_adam_msg_dds__HandState_,
    pnd_adam_msg_dds__LowCmd_,
    pnd_adam_msg_dds__LowState_,
)


def _model():
    return mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)


def _adapter(model):
    num_motor = model.nu - 24
    return PndAdapter(
        model,
        load_motor_mapping(),
        load_hand_mapping(),
        num_motor,
    )


def test_motor_mapping_uses_official_sdk_order():
    model = _model()
    adapter = _adapter(model)

    assert adapter.sdk_to_ctrl[12] == 14
    assert adapter.sdk_to_ctrl[13] == 12
    assert adapter.sdk_to_ctrl[14] == 13
    assert adapter.sdk_to_ctrl[15] == 15
    assert adapter.sdk_to_ctrl[19] == 21
    assert adapter.sdk_to_ctrl[21] == 19
    assert adapter.sdk_to_ctrl[26] == 28
    assert adapter.sdk_to_ctrl[28] == 26
    assert adapter.sdk_to_ctrl[29] == 29
    assert adapter.sdk_to_ctrl[30] == 30

    def actuator_name(ctrl_id):
        return mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, ctrl_id
        )

    assert actuator_name(adapter.sdk_to_ctrl[15]) == "shoulderPitch_Left"
    assert actuator_name(adapter.sdk_to_ctrl[29]) == "neckYaw"
    assert actuator_name(adapter.sdk_to_ctrl[26]) == "wristRoll_Right"


def test_lowcmd_writes_mapped_ctrl_and_moves_left_shoulder():
    model = _model()
    adapter = _adapter(model)
    data = mujoco.MjData(model)

    cmd = pnd_adam_msg_dds__LowCmd_(31)
    cmd.motor_cmd[15].mode = 1
    cmd.motor_cmd[15].tau = 20.0
    cmd.motor_cmd[15].kp = 0.0
    cmd.motor_cmd[15].kd = 0.0

    adapter.ApplyLowCmd(cmd, data, now=1.0)

    assert data.ctrl[15] == 20.0
    assert data.ctrl[29] == 0.0

    q_before = float(data.qpos[22])
    mujoco.mj_step(model, data)
    q_after = float(data.qpos[22])
    assert q_before != q_after


def test_lowstate_contains_standardized_fields():
    model = _model()
    adapter = _adapter(model)
    data = mujoco.MjData(model)
    data.time = 1.25
    mujoco.mj_forward(model, data)

    cmd = pnd_adam_msg_dds__LowCmd_(31)
    cmd.mode_pr = 1
    cmd.motor_cmd[0].mode = 1
    adapter.ApplyLowCmd(cmd, data, now=1.0)

    state = pnd_adam_msg_dds__LowState_(31)
    adapter.FillLowState(state, data)

    assert state.tick == 1250
    assert state.mode_pr == 1
    assert state.motor_state[0].mode == 1
    assert state.motor_state[0].state == 1
    assert state.motor_state[0].ddq == float(data.qacc[6])
    assert state.battery_data.status == "SIM_NO_BATTERY_MODEL"


def test_handstate_comes_from_finger_qpos():
    model = _model()
    adapter = _adapter(model)
    data = mujoco.MjData(model)

    for group in adapter.hand_groups:
        for qpos_id in group["qpos"]:
            if group["kind"] in ("thumb", "thumb_lateral"):
                data.qpos[qpos_id] = 1.0
            else:
                data.qpos[qpos_id] = 1.6
    mujoco.mj_forward(model, data)

    state = pnd_adam_msg_dds__HandState_()
    adapter.FillHandState(state, data)

    assert state.position[0] == 0
    assert state.position[4] == 0
    assert state.position[5] == 0

    for qpos_id in [q for group in adapter.hand_groups for q in group["qpos"]]:
        data.qpos[qpos_id] = 0.0
    mujoco.mj_forward(model, data)
    adapter.FillHandState(state, data)

    assert state.position[0] == 1000
    assert state.position[4] == 1000
    assert state.position[5] == 1000


def test_watchdog_damps_motors_and_holds_hands():
    model = _model()
    adapter = _adapter(model)
    data = mujoco.MjData(model)

    cmd = pnd_adam_msg_dds__LowCmd_(31)
    cmd.motor_cmd[0].mode = 1
    cmd.motor_cmd[0].kd = 2.0
    adapter.ApplyLowCmd(cmd, data, now=1.0)

    dof_id = adapter.ctrl_to_dof[0]
    data.qvel[dof_id] = 3.0

    for ctrl_id in adapter.hand_ctrl_ids:
        qpos_id = adapter.ctrl_to_qpos[ctrl_id]
        data.qpos[qpos_id] = 0.7

    adapter.ApplyWatchdog(data, now=1.2)

    assert adapter.motor_active is False
    assert data.ctrl[0] == -6.0
    for ctrl_id in adapter.hand_ctrl_ids:
        qpos_id = adapter.ctrl_to_qpos[ctrl_id]
        assert data.ctrl[ctrl_id] == data.qpos[qpos_id]


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
