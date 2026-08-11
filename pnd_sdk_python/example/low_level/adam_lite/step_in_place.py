import time
import sys
import numpy as np
from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelSubscriber
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowCmd_
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowState_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowCmd_, LowState_
from pndbotics_sdk_py.utils.thread import RecurrentThread

ADAM_NUM_MOTOR = 23

# 从原示例中复制的 Kp 和 Kd 数组
Kp = [
    305.0, 700.0, 405.0, # hip left
    305.0, 20, 0, #knee and ankle
    305.0, 700.0, 405.0, # hip right
    305.0, 20, 0, #knee and ankle
    405.0, 405.0, 205.0, # waist
    18.0, 9.0, 9.0, # shoulder left
    9.0,  # arms
    18.0, 9.0, 9.0, # shoulder right
    9.0 # arms
]

Kd = [
    6.1, 30.0, 6.1,
    6.1, 2.5, 0.35,
    6.1, 30.0, 6.1,
    6.1, 2.5, 0.35,    # legs
    4.1, 6.1, 6.1,             # waist
    0.9, 0.9, 0.9,
    0.9,  # arms
    0.9, 0.9, 0.9, 
    0.9 # arms 
]

class ADAMJointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        
    WaistPitch = 14       
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    RightShoulderPitch = 19
    RightShoulderRoll = 20
    RightShoulderYaw = 21
    RightElbow = 22

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.001
        self.low_cmd = pnd_adam_msg_dds__LowCmd_(ADAM_NUM_MOTOR)
        self.low_state = pnd_adam_msg_dds__LowState_(ADAM_NUM_MOTOR)
        # 踏步相位控制
        self.phase = 0          # 0: 左腿抬起, 1: 右腿抬起
        self.phase_time = 0.0   # 当前相位已持续的时间
        self.phase_duration = 0.8  # 每步持续 0.8 秒

    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

    def LowCmdWrite(self):
        self.time_ += self.control_dt_
        self.phase_time += self.control_dt_

        # 相位切换
        if self.phase_time >= self.phase_duration:
            self.phase = 1 - self.phase  # 0<->1 切换
            self.phase_time = 0.0

        # 根据相位设定目标角度（弧度）
        if self.phase == 0:  # 左腿抬起，右腿放下
            hip_left_target = -0.6
            knee_left_target = 0.8
            hip_right_target = 0.0
            knee_right_target = 0.0
        else:               # 右腿抬起，左腿放下
            hip_left_target = 0.0
            knee_left_target = 0.0
            hip_right_target = -0.6
            knee_right_target = 0.8

        # 填充所有电机的命令（除控制关节外，其余设为零位）
        for i in range(ADAM_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1          # 使能
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp[i]
            self.low_cmd.motor_cmd[i].kd = Kd[i]
            # 默认目标为零，稍后单独覆盖四个关节
            self.low_cmd.motor_cmd[i].q = 0.0

        # 设置四个关节的目标位置
        self.low_cmd.motor_cmd[ADAMJointIndex.LeftHipPitch].q = hip_left_target
        self.low_cmd.motor_cmd[ADAMJointIndex.LeftKnee].q = knee_left_target
        self.low_cmd.motor_cmd[ADAMJointIndex.RightHipPitch].q = hip_right_target
        self.low_cmd.motor_cmd[ADAMJointIndex.RightKnee].q = knee_right_target

        # ===== 新增：强制偏航和滚动关节为 0，防止旋转 =====
        self.low_cmd.motor_cmd[ADAMJointIndex.LeftHipYaw].q = 0.0
        self.low_cmd.motor_cmd[ADAMJointIndex.RightHipYaw].q = 0.0
        self.low_cmd.motor_cmd[ADAMJointIndex.WaistYaw].q = 0.0
        self.low_cmd.motor_cmd[ADAMJointIndex.LeftHipRoll].q = 0.0
        self.low_cmd.motor_cmd[ADAMJointIndex.RightHipRoll].q = 0.0
        self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleRoll].q = 0.0
        self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleRoll].q = 0.0
        # ==================================================

        # 发布命令
        self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    # 使用 domain id = 1，interface = "lo"（与 config.py 一致）
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(1, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    custom = Custom()
    custom.Init()
    print("Initialized")
    custom.Start()

    while True:
        time.sleep(1)
