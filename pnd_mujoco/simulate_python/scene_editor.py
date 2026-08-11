#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MuJoCo patient_forearm 场景编辑器

功能：
  1. 加载 adam_proximity_scene.xml
  2. patient_forearm 作为 mocap body，可用 MuJoCo 鼠标扰动直接移动/旋转
  3. 按 S：自动把当前位姿写回 XML
  4. 按 R：恢复 XML 中保存的位姿
  5. 按 Q / ESC：退出

运行：
  python3 scene_editor.py
"""

import math
import os
import re
import time
import threading

import mujoco
import mujoco.viewer


# ============================================================
# 配置
# ============================================================

SCENE_XML = os.path.expanduser(
    "~/pnd_mujoco/simulate_python/adam_proximity_scene.xml"
)

BODY_NAME = "patient_forearm"


# ============================================================
# 键盘状态
# ============================================================

save_requested = False
reload_requested = False
quit_requested = False

state_lock = threading.Lock()


# ============================================================
# 数学：MuJoCo quaternion [w,x,y,z] -> XYZ Euler(rad)
# ============================================================

def quat_to_euler_xyz(q):
    w, x, y, z = [float(v) for v in q]

    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + z * w)
    r20 = 2.0 * (x * z - y * w)

    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    ry = math.asin(max(-1.0, min(1.0, -r20)))
    cy = math.cos(ry)

    if abs(cy) > 1e-8:
        rx = math.atan2(r21, r22)
        rz = math.atan2(r10, r00)
    else:
        # 万向节锁退化情况
        r11 = 1.0 - 2.0 * (x * x + z * z)
        r12 = 2.0 * (y * z - x * w)
        rx = math.atan2(-r12, r11)
        rz = 0.0

    return rx, ry, rz


# ============================================================
# 数学：XYZ Euler(rad) -> MuJoCo quaternion [w,x,y,z]
# ============================================================

def euler_xyz_to_quat(euler):
    rx, ry, rz = [float(v) for v in euler]

    cx = math.cos(rx * 0.5)
    sx = math.sin(rx * 0.5)

    cy = math.cos(ry * 0.5)
    sy = math.sin(ry * 0.5)

    cz = math.cos(rz * 0.5)
    sz = math.sin(rz * 0.5)

    # R = Rx * Ry * Rz
    w = cx * cy * cz - sx * sy * sz
    x = sx * cy * cz + cx * sy * sz
    y = cx * sy * cz - sx * cy * sz
    z = cx * cy * sz + sx * sy * cz

    norm = math.sqrt(w*w + x*x + y*y + z*z)

    return [
        w / norm,
        x / norm,
        y / norm,
        z / norm,
    ]


def format_vec(values):
    return " ".join(f"{float(v):.9f}" for v in values)


# ============================================================
# XML：找到 patient_forearm 的 body 开始标签
# ============================================================

def find_forearm_body_tag(xml):
    pattern = (
        r'<body\s+'
        r'(?=[^>]*\bname\s*=\s*"patient_forearm")'
        r'[^>]*>'
    )

    match = re.search(pattern, xml, flags=re.DOTALL)

    if not match:
        raise RuntimeError(
            'XML 中没有找到 <body name="patient_forearm">'
        )

    return match


# ============================================================
# XML：更新/添加属性
# ============================================================

def update_attr(tag, attr_name, value):
    pattern = rf'\b{re.escape(attr_name)}\s*=\s*"[^"]*"'

    if re.search(pattern, tag):
        return re.sub(
            pattern,
            f'{attr_name}="{value}"',
            tag,
            count=1,
        )

    return tag[:-1] + f' {attr_name}="{value}">'


# ============================================================
# 保存当前位姿
# ============================================================

def save_forearm_pose(pos, quat):

    rx, ry, rz = quat_to_euler_xyz(quat)

    with open(SCENE_XML, "r", encoding="utf-8") as f:
        xml = f.read()

    match = find_forearm_body_tag(xml)

    old_tag = match.group(0)
    new_tag = old_tag

    new_tag = update_attr(
        new_tag,
        "pos",
        format_vec(pos),
    )

    new_tag = update_attr(
        new_tag,
        "euler",
        format_vec((rx, ry, rz)),
    )

    # 确保它仍然是 mocap body
    new_tag = update_attr(
        new_tag,
        "mocap",
        "true",
    )

    new_xml = (
        xml[:match.start()]
        + new_tag
        + xml[match.end():]
    )

    # 临时文件 + 原子替换
    tmp = SCENE_XML + ".tmp"

    with open(tmp, "w", encoding="utf-8") as f:
        f.write(new_xml)

    os.replace(tmp, SCENE_XML)

    print()
    print("=" * 60)
    print("已保存 patient_forearm")
    print("=" * 60)
    print("pos:")
    print("  ", format_vec(pos))
    print("euler(rad):")
    print("  ", format_vec((rx, ry, rz)))
    print("euler(deg):")
    print(
        "  ",
        format_vec([
            math.degrees(rx),
            math.degrees(ry),
            math.degrees(rz),
        ])
    )
    print("=" * 60)
    print()


# ============================================================
# 从 XML 读取保存的位姿
# ============================================================

def read_saved_pose():

    with open(SCENE_XML, "r", encoding="utf-8") as f:
        xml = f.read()

    match = find_forearm_body_tag(xml)

    tag = match.group(0)

    pos_match = re.search(
        r'\bpos\s*=\s*"([^"]*)"',
        tag,
    )

    euler_match = re.search(
        r'\beuler\s*=\s*"([^"]*)"',
        tag,
    )

    if not pos_match or not euler_match:
        raise RuntimeError(
            "patient_forearm 缺少 pos 或 euler"
        )

    pos = [
        float(v)
        for v in pos_match.group(1).split()
    ]

    euler = [
        float(v)
        for v in euler_match.group(1).split()
    ]

    if len(pos) != 3 or len(euler) != 3:
        raise RuntimeError(
            "pos/euler 必须各包含 3 个数字"
        )

    return pos, euler


# ============================================================
# 键盘回调
# ============================================================

def key_callback(key):

    global save_requested
    global reload_requested
    global quit_requested

    # S
    if key in (ord("S"), ord("s")):
        with state_lock:
            save_requested = True

    # R
    elif key in (ord("R"), ord("r")):
        with state_lock:
            reload_requested = True

    # Q / ESC
    elif key in (ord("Q"), ord("q"), 256):
        with state_lock:
            quit_requested = True


# ============================================================
# 主程序
# ============================================================

def main():

    global save_requested
    global reload_requested
    global quit_requested

    if not os.path.isfile(SCENE_XML):
        raise FileNotFoundError(
            f"\n找不到场景文件：\n{SCENE_XML}\n"
        )

    print("=" * 60)
    print("MuJoCo patient_forearm 场景编辑器")
    print("=" * 60)
    print(f"Scene: {SCENE_XML}")
    print()
    print("操作：")
    print("  选择 patient_forearm")
    print("  使用 MuJoCo 鼠标扰动直接移动/旋转")
    print("  S       保存当前位姿")
    print("  R       恢复 XML 中的位姿")
    print("  Q/ESC   退出")
    print()
    print("XML 使用 angle='radian'")
    print("所以保存到 XML 的 euler 是弧度。")
    print("=" * 60)

    # --------------------------------------------------------
    # 加载模型
    # --------------------------------------------------------

    model = mujoco.MjModel.from_xml_path(
        SCENE_XML
    )

    data = mujoco.MjData(model)

    # --------------------------------------------------------
    # 找 patient_forearm
    # --------------------------------------------------------

    body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        BODY_NAME,
    )

    if body_id < 0:
        raise RuntimeError(
            f"找不到 body：{BODY_NAME}"
        )

    # --------------------------------------------------------
    # 检查是否 mocap body
    # --------------------------------------------------------

    mocap_id = model.body_mocapid[body_id]

    if mocap_id < 0:
        raise RuntimeError(
            "\n"
            "patient_forearm 当前不是 mocap body。\n\n"
            "XML 必须类似：\n\n"
            '<body\n'
            '    name="patient_forearm"\n'
            '    mocap="true"\n'
            '    pos="0.8 -0.15 0.85"\n'
            '    euler="-90 -90 -90">\n'
        )

    mujoco.mj_forward(model, data)

    # --------------------------------------------------------
    # 启动 MuJoCo viewer
    # --------------------------------------------------------

    viewer = mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
    )

    try:

        viewer.cam.azimuth = model.vis.global_.azimuth
        viewer.cam.elevation = model.vis.global_.elevation
        viewer.cam.distance = 3.0
        viewer.cam.lookat[:] = [0.8, 0.0, 0.9]

        # 显示 site
        viewer.opt.sitegroup[0] = 1

        print()
        print("MuJoCo 编辑器已经启动。")
        print("请选择 patient_forearm 并进行鼠标拖动。")
        print("调整满意后按 S 保存。")
        print()

        # ----------------------------------------------------
        # 主循环
        # ----------------------------------------------------

        while viewer.is_running():

            with state_lock:
                do_save = save_requested
                do_reload = reload_requested
                do_quit = quit_requested

                save_requested = False
                reload_requested = False
                quit_requested = False

            # ------------------------------------------------
            # 保存
            # ------------------------------------------------

            if do_save:

                pos = data.mocap_pos[
                    mocap_id
                ].copy()

                quat = data.mocap_quat[
                    mocap_id
                ].copy()

                try:
                    save_forearm_pose(
                        pos,
                        quat,
                    )

                except Exception as e:
                    print(
                        f"\n保存失败：{e}\n"
                    )

            # ------------------------------------------------
            # 从 XML 恢复
            # ------------------------------------------------

            if do_reload:

                try:

                    saved_pos, saved_euler = (
                        read_saved_pose()
                    )

                    saved_quat = (
                        euler_xyz_to_quat(
                            saved_euler
                        )
                    )

                    data.mocap_pos[
                        mocap_id
                    ] = saved_pos

                    data.mocap_quat[
                        mocap_id
                    ] = saved_quat

                    mujoco.mj_forward(
                        model,
                        data,
                    )

                    print()
                    print(
                        "已恢复 XML 中保存的"
                        " patient_forearm 位姿。"
                    )
                    print()

                except Exception as e:
                    print(
                        f"\n恢复失败：{e}\n"
                    )

            if do_quit:
                break

            # ------------------------------------------------
            # 物理推进
            # ------------------------------------------------

            mujoco.mj_step(
                model,
                data,
            )

            # passive viewer 必须同步
            viewer.sync()

            time.sleep(
                model.opt.timestep
            )

    finally:

        viewer.close()

    print("编辑器已退出。")


if __name__ == "__main__":
    main()
