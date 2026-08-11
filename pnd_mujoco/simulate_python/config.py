ROBOT = "adam_pro"

ROBOT_SCENE = "adam_proximity_scene.xml"


HANDPOSE_SRC = 1


# -----------------------------
# DDS / ROS2
# -----------------------------

# 保留PND官方DDS通信
SDK_TYPE = "DDS"

DOMAIN_ID = 1


# DDS网卡
INTERFACE = "lo"


# ROS2状态发布由独立节点 pnd_joint_state_publisher 负责
ENABLE_ROS2_OUTPUT = False



# -----------------------------
# Viewer
# -----------------------------

USE_JOYSTICK = 0

JOYSTICK_TYPE = "xbox"

JOYSTICK_DEVICE = 0


PRINT_SCENE_INFORMATION = False

ENABLE_ELASTIC_BAND = True


SIMULATE_DT = 0.001

VIEWER_DT = 0.01
