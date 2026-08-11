import time

from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelSubscriber
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandState_


LEFT_HAND_NAMES = [
    "Pinky",
    "Ring",
    "Middle",
    "Index",
    "Thumb",
    "ThumbLat",
]

RIGHT_HAND_NAMES = [
    "Pinky",
    "Ring",
    "Middle",
    "Index",
    "Thumb",
    "ThumbLat",
]


ChannelFactoryInitialize(1, "lo")

sub = ChannelSubscriber("rt/handstate", HandState_)
sub.Init()

print("waiting rt/handstate...")

count = 0

while True:
    msg = sub.Read()

    if msg:
        count += 1

        if count == 1 or count % 100 == 0:
            print("=============================")
            print("Hand State")
            print("=============================")
            print("Left")
            for i, name in enumerate(LEFT_HAND_NAMES):
                print(f"{name:<12}{msg.position[i]}")
            print()
            print("Right")
            for i, name in enumerate(RIGHT_HAND_NAMES):
                print(f"{name:<12}{msg.position[6 + i]}")
            print()

    time.sleep(0.001)
