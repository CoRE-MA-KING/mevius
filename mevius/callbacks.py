"""Callback functions for MEVIUS."""

import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .mevius_utils.parameters import parameters as P
from .types import ModeCommand, PeripheralState, RobotCommand, RobotState


def command_callback(command: ModeCommand, robot_state: RobotState, robot_command: RobotCommand):
    print('command_callback')
    print([command, robot_state, robot_command])
    with robot_command.lock:
        prev_command = robot_command.command
        if not robot_command.initialized:
            pass
    if command == 'STANDBY-STANDUP':
        with robot_command.lock:
            if robot_command.remaining_time < 0.1:
                if prev_command == 'STANDBY':
                    robot_command.command = 'STANDUP'
                    with robot_state.lock:
                        robot_command.initial_angle = robot_state.angle[:]
                        robot_command.final_angle = P.DEFAULT_ANGLE[:]
                        robot_command.interpolating_time = 3.0
                        robot_command.remaining_time = robot_command.interpolating_time
                elif prev_command == 'STANDUP':
                    robot_command.command = 'STANDBY'
                    with robot_state.lock:
                        robot_command.initial_angle = robot_state.angle[:]
                        robot_command.final_angle = P.STANDBY_ANGLE[:]
                        robot_command.interpolating_time = 3.0
                        robot_command.remaining_time = robot_command.interpolating_time
    elif command == 'STANDUP-WALK':
        with robot_command.lock:
            if robot_command.remaining_time < 0.1:
                if prev_command == 'STANDUP':
                    robot_command.command = 'WALK'
                    robot_command.interpolating_time = 3.0
                    robot_command.remaining_time = robot_command.interpolating_time
                elif prev_command == 'WALK':
                    robot_command.command = 'STANDUP'
                    with robot_state.lock:
                        robot_command.initial_angle = robot_state.angle[:]
                        robot_command.final_angle = P.DEFAULT_ANGLE[:]
                        robot_command.interpolating_time = 3.0
                        robot_command.remaining_time = robot_command.interpolating_time
    elif command == 'STANDBY':
        with robot_command.lock:
            robot_command.command = 'STANDBY'
            with robot_state.lock:
                robot_command.initial_angle = robot_state.angle[:]
                robot_command.final_angle = P.STANDBY_ANGLE[:]
                robot_command.interpolating_time = 3.0
                robot_command.remaining_time = robot_command.interpolating_time
    elif command == 'STANDUP':
        robot_command.command = 'STANDUP'
        with robot_state.lock:
            robot_command.initial_angle = robot_state.angle[:]
            robot_command.final_angle = P.DEFAULT_ANGLE[:]
            robot_command.interpolating_time = 3.0
            robot_command.remaining_time = robot_command.interpolating_time
    elif command == 'DEBUG':
        robot_command.command = 'DEBUG'
        with robot_state.lock:
            robot_command.initial_angle = robot_state.angle[:]
            robot_command.final_angle = P.DEBUG_ANGLE[:]
            robot_command.interpolating_time = 3.0
            robot_command.remaining_time = robot_command.interpolating_time
    elif prev_command == 'STANDUP' and command == 'WALK':
        robot_command.command = 'WALK'

    with robot_command.lock:
        print('Command changed from {} to {}'.format(prev_command, robot_command.command))


def realsense_vel_callback(msg: Odometry, params: PeripheralState):
    peripherals_state = params
    print('realsense vel callback!')
    with peripherals_state.lock:
        peripherals_state.body_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        # get odom quat
        peripherals_state.body_quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        peripherals_state.realsense_last_time = time.time()
        print(peripherals_state.realsense_last_time)
        print('               UPDATE !! REALSENSE!!!!')


def realsense_gyro_callback(msg: Imu, params: PeripheralState):
    peripherals_state = params
    with peripherals_state.lock:
        # peripherals_state.body_gyro = [
        #     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        # ]
        # for realsenes arrangement
        peripherals_state.body_gyro = [
            msg.angular_velocity.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
        ]


def realsense_acc_callback(msg: Imu, params: PeripheralState):
    peripherals_state = params
    with peripherals_state.lock:
        # peripherals_state.body_acc = [
        #     msg.linear_acceleration.x, msg.linear_acceleration.y, -msg.linear_acceleration.z
        # ]
        # for realsenes arrangement
        peripherals_state.body_acc = [
            msg.linear_acceleration.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
        ]
