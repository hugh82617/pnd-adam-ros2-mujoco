"""Build standard sensor_msgs/Imu from PND LowState IMU."""

from typing import Any

from sensor_msgs.msg import Imu


def build_imu_msg(stamp: Any, low_state: Any) -> Imu:
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = "imu_link"

    quat = low_state.imu_state.quaternion
    msg.orientation.w = float(quat[0])
    msg.orientation.x = float(quat[1])
    msg.orientation.y = float(quat[2])
    msg.orientation.z = float(quat[3])

    gyro = low_state.imu_state.gyroscope
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])

    acc = low_state.imu_state.accelerometer
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    return msg
