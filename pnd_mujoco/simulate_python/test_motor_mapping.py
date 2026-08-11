import time

from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelSubscriber

from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_



motor_names = [

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

"neckYaw",
"neckPitch"

]



ChannelFactoryInitialize(
    1,
    "lo"
)


sub = ChannelSubscriber(
    "rt/lowstate",
    LowState_
)

sub.Init()


print("waiting...")


while True:

    msg=sub.Read()

    if msg:

        print("\n========= Joint State =========")

        for i,motor in enumerate(msg.motor_state):

            print(
                i,
                motor_names[i],
                "q=",
                motor.q
            )


        break


    time.sleep(0.001)
