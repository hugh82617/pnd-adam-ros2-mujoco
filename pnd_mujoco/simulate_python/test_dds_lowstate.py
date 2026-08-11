import time

from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelSubscriber

from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_

# ----------------------------------------
# Adam Pro 31个关节名称
# ----------------------------------------
joint_names = [
    "hipPitch_Left",
    "hipRoll_Left",
    "hipYaw_Left",
    "kneePitch_Left",
    "anklePitch_Left",
    "ankleRoll_Left",

    "hipPitch_Right",
    "hipRoll_Right",
    "hipYaw_Right",
    "kneePitch_Right",
    "anklePitch_Right",
    "ankleRoll_Right",

    "waistRoll",
    "waistPitch",
    "waistYaw",

    "neckYaw",
    "neckPitch",

    "shoulderPitch_Left",
    "shoulderRoll_Left",
    "shoulderYaw_Left",
    "elbow_Left",
    "wristYaw_Left",
    "wristPitch_Left",
    "wristRoll_Left",

    "shoulderPitch_Right",
    "shoulderRoll_Right",
    "shoulderYaw_Right",
    "elbow_Right",
    "wristYaw_Right",
    "wristPitch_Right",
    "wristRoll_Right",
]

# ----------------------------------------
# 初始化DDS
# ----------------------------------------
ChannelFactoryInitialize(
    1,
    "lo"
)

# ----------------------------------------
# 创建Subscriber
# ----------------------------------------
sub = ChannelSubscriber(
    "rt/lowstate",
    LowState_
)

sub.Init()

print("Waiting rt/lowstate ...")

count = 0

while True:

    msg = sub.Read()

    if msg is not None:

        count += 1

        # 每100帧打印一次
        if count % 100 == 0:

            print("\n")
            print("=" * 80)
            print("Adam Pro LowState")
            print("=" * 80)

            print("Motor Number :", len(msg.motor_state))

            print("\n================ Robot Joints =================")

            for i, motor in enumerate(msg.motor_state):

                if i < len(joint_names):
                    name = joint_names[i]
                else:
                    name = f"motor{i}"

                print(
                    f"{i:02d} "
                    f"{name:<22}"
                    f"q={motor.q:9.4f}   "
                    f"dq={motor.dq:9.4f}   "
                    f"tau={motor.tau_est:9.4f}   "
                    f"mode={motor.mode}"
                )

            print("\n================ IMU =================")

            try:
                print("Quaternion :",
                      [round(x, 5) for x in msg.imu_state.quaternion])

                print("Gyroscope  :",
                      [round(x, 5) for x in msg.imu_state.gyroscope])

                print("Accel      :",
                      [round(x, 5) for x in msg.imu_state.accelerometer])

            except Exception:
                print("IMU not available.")

            print("\n================ Wireless Remote =================")

            try:
                print(list(msg.wireless_remote))
            except Exception:
                print("Wireless Remote unavailable.")

            print("=" * 80)

    time.sleep(0.001)