"""Keyboard Joy node for MEVIUS."""

from functools import partial

from rclpy.node import Node
from sensor_msgs.msg import Joy

from ..callbacks import command_callback
from ..types import PeripheralState, RobotCommand, RobotState


class KeyboardJoy(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripheral_state: PeripheralState,
    ):
        super().__init__('key_joy_node')
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripheral_state = peripheral_state
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            partial(self.virtual_joy_callback, params=(self.peripheral_state)),
            1,
        )
        self.subscription

    def virtual_joy_callback(self, msg: Joy, params: PeripheralState):
        peripherals_state = params
        # print(msg)
        with peripherals_state.lock:
            peripherals_state.virtual_enable = True
            peripherals_state.virtual = [
                msg.axes[0],
                msg.axes[1],
                msg.axes[3],
                msg.buttons[0],
                msg.buttons[1],
            ]
            one_pushed = peripherals_state.virtual[3]
            two_pushed = peripherals_state.virtual[4]

        if one_pushed == 1:
            command_callback('STANDBY-STANDUP', self.robot_state, self.robot_command)
        elif two_pushed == 1:
            command_callback('STANDUP-WALK', self.robot_state, self.robot_command)
