"""Mevius command node."""

from functools import partial
from typing import Tuple

from rclpy.node import Node
from std_msgs.msg import String

from ..callbacks import command_callback
from ..types import RobotCommand, RobotState


class MeviusCommand(Node):
    def __init__(self, robot_state: RobotState, robot_command: RobotCommand):
        super().__init__('mevius_command')
        # self.subscription=self.create_subscription(String, ros_command_callback, (robot_state, robot_command), queue_size=1))
        # rospy.Subscriber('/mevius_command', String, ros_command_callback, (robot_state, robot_command), queue_size=1)

        self.robot_state = robot_state
        self.robot_command = robot_command

        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            String,  # メッセージの型
            'mevius_command',  # トピック名
            partial(self.ros_command_callback, params=(robot_state, robot_command)),
            1,  # キューサイズ
        )
        self.subscription  # サブスクライバーを保持（破棄されないように）

    def ros_command_callback(
        self, msg: String, params: Tuple[RobotState, RobotCommand]
    ):
        robot_state, robot_command = params
        print('Received ROS Command: {}'.format(msg.data))
        command_callback(msg.data, robot_state, robot_command)
