"""Camera subscriber nodes for MEVIUS."""

from functools import partial

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from ..callbacks import (
    realsense_acc_callback,
    realsense_gyro_callback,
    realsense_vel_callback,
)
from ..types import PeripheralState


class CameraOdom(Node):
    def __init__(self, peripheral_state: PeripheralState):
        super().__init__("odom")

        # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Odometry,
            "/camera/pose/sample",
            partial(realsense_vel_callback, params=(peripheral_state)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscription


class CameraGyro(Node):
    def __init__(self, peripheral_state: PeripheralState):
        super().__init__("gyro")

        self.subscription = self.create_subscription(
            Imu,
            "camera/gyro/sample",
            partial(realsense_gyro_callback, params=(peripheral_state)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscription


class CameraAccel(Node):
    def __init__(self, peripheral_state: PeripheralState):
        super().__init__("accel")

        self.subscription = self.create_subscription(
            Imu,
            "camera/accel/sample",
            partial(realsense_acc_callback, params=(peripheral_state)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscription
