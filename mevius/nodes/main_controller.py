"""Main controller node for MEVIUS."""

import os
import time

import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from ..mevius_utils import (
    get_policy_observation,
    get_policy_output,
    get_urdf_joint_params,
    read_torch_policy,
)
from ..mevius_utils.parameters import parameters as P
from ..types import PeripheralState, RobotCommand, RobotState


class MainController(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripherals_state: PeripheralState,
    ):
        super().__init__("main_controller")
        print("Init main_controller Node")
        self.controlrate = 50.0
        self.timer = self.create_timer(1.0 / self.controlrate, self.timer_callback)
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripherals_state = peripherals_state
        policy_path = os.path.join(
            get_package_share_directory("mevius"), "models/policy_slow.pt"
        )
        self.policy = read_torch_policy(policy_path).to("cpu")

        urdf_fullpath = os.path.join(
            get_package_share_directory("mevius"), "models/mevius.urdf"
        )
        self.joint_params = get_urdf_joint_params(urdf_fullpath, P.JOINT_NAME)

        self.is_safe = True
        self.last_actions = [0.0] * 12  # TODO initialize

    def timer_callback(self):
        # rate = rospy.Rate(P.CONTROL_HZ)
        # while rclpy.ok():
        with self.robot_command.lock:
            command = self.robot_command.command
        if command in ["STANDBY", "STANDUP", "DEBUG"]:
            with self.robot_command.lock:
                self.robot_command.remaining_time -= 1.0 / self.controlrate
                self.robot_command.remaining_time = max(
                    0, self.robot_command.remaining_time
                )
                if self.robot_command.remaining_time <= 0:
                    pass
                else:
                    ratio = (
                        1
                        - self.robot_command.remaining_time
                        / self.robot_command.interpolating_time
                    )
                    self.robot_command.angle = [
                        a + (b - a) * ratio
                        for a, b in zip(
                            self.robot_command.initial_angle,
                            self.robot_command.final_angle,
                        )
                    ]
        elif command in ["WALK"]:
            with self.robot_command.lock:
                self.robot_command.remaining_time -= 1.0 / self.controlrate
                self.robot_command.remaining_time = max(
                    0, self.robot_command.remaining_time
                )

            with self.peripherals_state.lock:
                base_quat = self.peripherals_state.body_quat[:]
                base_lin_vel = self.peripherals_state.body_vel[:]
                base_ang_vel = self.peripherals_state.body_gyro[:]

                ranges = P.commands.ranges
                coefs = [
                    ranges.lin_vel_x[1],
                    ranges.lin_vel_y[1],
                    ranges.ang_vel_yaw[1],
                    ranges.heading[1],
                ]
                if self.peripherals_state.spacenav_enable:
                    nav = self.peripherals_state.spacenav[:]
                    max_command = 0.6835
                    commands_ = [nav[0], nav[1], nav[5], nav[5]]
                    commands = [
                        [
                            min(max(-coef, coef * command / max_command), coef)
                            for coef, command in zip(coefs, commands_)
                        ]
                    ]
                elif self.peripherals_state.virtual_enable:
                    nav = self.peripherals_state.virtual[:]
                    max_command = 1.0
                    x_vel = nav[1]
                    y_vel = nav[0]
                    yaw_vel = nav[2]
                    commands_ = [x_vel, y_vel, yaw_vel, yaw_vel]
                    commands = [
                        [
                            min(max(-coef, coef * command / max_command), coef)
                            for coef, command in zip(coefs, commands_)
                        ]
                    ]
                else:
                    commands = torch.tensor(
                        [[0.0, 0.0, 0.0, 0.0]], dtype=torch.float, requires_grad=False
                    )

                print("High Level Commands: {}".format(commands))

        # for safety
        if command in ["WALK"]:
            # no realsense
            with self.peripherals_state.lock:
                if self.peripherals_state.realsense_last_time is None:
                    self.is_safe = False
                    print("No Connection to Realsense. PD gains become 0.")
                if (self.peripherals_state.realsense_last_time is not None) and (
                    time.time() - self.peripherals_state.realsense_last_time > 0.1
                ):
                    print("Realsense data is too old. PD gains become 0.")
                    self.is_safe = False
            # falling down
            if self.is_safe and (Rotation.from_quat(base_quat).as_matrix()[2, 2] < 0.6):
                self.is_safe = False
                print("Robot is almost fell down. PD gains become 0.")

            # self.is_safe=True
            if not self.is_safe:
                print("Robot is not safe. Please reboot the robot.")
                with self.robot_command.lock:
                    self.robot_command.kp = [0.0] * 12
                    self.robot_command.kd = [0.0] * 12
                    with self.robot_state.lock:
                        self.robot_command.angle = self.robot_state.angle[:]
                # rate.sleep()
                # continue

        if command in ["WALK"]:
            with self.robot_state.lock:
                dof_pos = self.robot_state.angle[:]
                dof_vel = self.robot_state.velocity[:]
            # print(base_quat, base_lin_vel, base_ang_vel, commands, dof_pos, dof_vel, last_actions)
            obs = get_policy_observation(
                base_quat,
                base_lin_vel,
                base_ang_vel,
                commands,
                dof_pos,
                dof_vel,
                self.last_actions,
            )
            actions = get_policy_output(self.policy, obs)
            scaled_actions = P.control.action_scale * actions

        if command in ["WALK"]:
            ref_angle = [a + b for a, b in zip(scaled_actions, P.DEFAULT_ANGLE[:])]
            with self.robot_state.lock:
                for i in range(len(ref_angle)):
                    if (
                        self.robot_state.angle[i] < self.joint_params[i][0]
                        or self.robot_state.angle[i] > self.joint_params[i][1]
                    ):
                        ref_angle[i] = max(
                            self.joint_params[i][0] + 0.1,
                            min(ref_angle[i], self.joint_params[i][1] - 0.1),
                        )
                        print(
                            "# Joint {} out of range: {:.3f}".format(
                                P.JOINT_NAME[i], self.robot_state.angle[i]
                            )
                        )
            with self.robot_command.lock:
                self.robot_command.angle = ref_angle

            self.last_actions = actions[:]
