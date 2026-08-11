import time

from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelPublisher
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__HandCmd_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandCmd_


ChannelFactoryInitialize(1, "lo")

pub = ChannelPublisher("rt/handcmd", HandCmd_)
pub.Init()

cmd = pnd_adam_msg_dds__HandCmd_()

# Left hand thumb is position[4]; all other 11 DOF stay at 0.
left_thumb_values = [
    0,
    300,
    600,
    900,
    1200,
    1500,
    1800,
    1500,
    1200,
    900,
    600,
    300,
]

print("publishing rt/handcmd with left thumb 0..1800..0")

while True:
    for value in left_thumb_values:
        cmd.position[4] = value
        pub.Write(cmd)
        time.sleep(0.1)
