import argparse
import os
import threading
import time
from functools import partial
from typing import Literal, Tuple

import mujoco
import mujoco_viewer
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from mevius_massage.msg._mevius_log import MeviusLog
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import String

from .callbacks import (
    command_callback,
    realsense_acc_callback,
    realsense_gyro_callback,
    realsense_vel_callback,
)
from .nodes.can_communication import CanCommunication
from .nodes.keyboard_joy import KeyboardJoy
from .nodes.main_controller import MainController
from .nodes.publisher import JointStatePub, MeviusLogPub
from .nodes.sim_communication import SimCommunication
from .types import ModeCommand, PeripheralState, RobotCommand, RobotState


class CameraOdom(Node):
    def __init__(self, peripheral_state):
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
    def __init__(self, peripheral_state):
        super().__init__("gyro")

        self.subscription = self.create_subscription(
            Imu,
            "camera/gyro/sample",
            partial(realsense_gyro_callback, params=(peripheral_state)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscription


class CameraAccel(Node):
    def __init__(self, peripheral_state):
        super().__init__("accel")

        self.subscription = self.create_subscription(
            Imu,
            "camera/accel/sample",
            partial(realsense_acc_callback, params=(peripheral_state)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscription


class MeviusCommand(Node):
    def __init__(self, robot_state: RobotState, robot_command: RobotCommand):
        super().__init__("mevius_command")
        # self.subscription=self.create_subscription(String, ros_command_callback, (robot_state, robot_command), queue_size=1))
        # rospy.Subscriber('/mevius_command', String, ros_command_callback, (robot_state, robot_command), queue_size=1)

        self.robot_state = robot_state
        self.robot_command = robot_command

        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            String,  # メッセージの型
            "mevius_command",  # トピック名
            partial(self.ros_command_callback, params=(robot_state, robot_command)),
            1,  # キューサイズ
        )
        self.subscription  # サブスクライバーを保持（破棄されないように）

    def ros_command_callback(
        self, msg: String, params: Tuple[RobotState, RobotCommand]
    ):
        robot_state, robot_command = params
        print("Received ROS Command: {}".format(msg.data))
        command_callback(msg.data, robot_state, robot_command)


class Mevius(Node):
    def __init__(self):
        super().__init__("mevius")
        print("Init Mevius Node")
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        pass
        # print('mevius callback!')


def main():
    import sys

    # print(sys.path)

    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="do simulation")
    args = parser.parse_args()

    print("Hello mevius!!")
    rclpy.init()
    try:
        mevius = Mevius()

        robot_state = RobotState()
        peripheral_state = PeripheralState()
        robot_command = RobotCommand()

        main_controller = MainController(robot_state, robot_command, peripheral_state)

        mevius_command = MeviusCommand(robot_state, robot_command)

        keyboard_joy = KeyboardJoy(robot_state, robot_command, peripheral_state)
        camera_odom = CameraOdom(peripheral_state)
        camera_gyro = CameraGyro(peripheral_state)
        camera_accel = CameraAccel(peripheral_state)

        if args.sim:
            communication_thread = SimCommunication(
                robot_state, robot_command, peripheral_state
            )
        else:
            communication_thread = CanCommunication(
                robot_state, robot_command, peripheral_state
            )

        executor = SingleThreadedExecutor()
        executor.add_node(communication_thread)
        executor.add_node(mevius)
        executor.add_node(mevius_command)
        executor.add_node(keyboard_joy)
        executor.add_node(main_controller)
        executor.add_node(camera_odom)
        executor.add_node(camera_gyro)
        executor.add_node(camera_accel)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            mevius.destroy_node()
            mevius_command.destroy_node()

    except KeyboardInterrupt:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
