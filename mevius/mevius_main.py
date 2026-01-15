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
from .nodes.main_controller import MainController
from .types import ModeCommand, PeripheralState, RobotCommand, RobotState

# def spacenav_joy_callback(msg: Joy, params: PeripheralState):
#     peripherals_state = params
#     with peripherals_state.lock:
#         peripherals_state.spacenav_enable = True
#         peripherals_state.spacenav = [msg.axes[0], msg.axes[1], msg.axes[2],
#                                       msg.axes[3], msg.axes[4], msg.axes[5], msg.buttons[0], msg.buttons[1]]
#         left_pushed = peripherals_state.spacenav[6]
#         right_pushed = peripherals_state.spacenav[7]

#     if left_pushed == 1:
#         command_callback("STANDBY-STANDUP", robot_state, robot_command)
#     elif right_pushed == 1:
#         command_callback("STANDUP-WALK", robot_state, robot_command)


class CanCommunication(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripherals_state: PeripheralState,
    ):
        super().__init__("can_communication")
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripherals_state = peripherals_state

        print("Init can node")

        self.device = "can0"
        self.motor_type = "AK70_10_V1p1"
        self.n_motor = 12
        self.motors = [
            CanMotorController(
                self.device,
                P.CAN_ID[i],
                motor_type=self.motor_type,
                motor_dir=P.MOTOR_DIR[i],
            )
            for i in range(self.n_motor)
        ]

        print("Enabling Motors...")
        for i, motor in enumerate(self.motors):
            pos, vel, cur, tem = motor.enable_motor()
            print(
                (
                    "Enabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, "
                    "Cur: {:.3f}, Temp: {:.3f}"
                ).format(P.JOINT_NAME[i], pos, vel, cur, tem)
            )
            with self.robot_state.lock:
                self.robot_state.angle[i] = pos
                self.robot_state.velocity[i] = vel
                self.robot_state.current[i] = cur
                self.robot_state.temperature[i] = tem
        print("Finish enabling motors!")
        self.state_pub = MeviusLogPub()
        self.jointstate_pub = JointStatePub()
        # state_pub = rospy.Publisher('mevius_log', MeviusLog, queue_size=2)
        # jointstate_pub = rospy.Publisher('joint_states', JointState, queue_size=2)

        print("Setting Initial Offset...")
        for i, motor in enumerate(self.motors):
            motor.set_angle_offset(P.STANDBY_ANGLE[i], deg=False)
            # motor.set_angle_range(joint_params[i][0], joint_params[i][1], deg=False)

        with self.robot_state.lock:
            self.robot_state.angle = P.STANDBY_ANGLE[:]

        with self.robot_command.lock:
            self.robot_command.command = "STANDBY"
            self.robot_command.angle = P.STANDBY_ANGLE[:]
            self.robot_command.initial_angle = P.STANDBY_ANGLE[:]
            self.robot_command.final_angle = P.STANDBY_ANGLE[:]
            self.robot_command.interpolating_time = 3.0
            self.robot_command.remaining_time = self.robot_command.interpolating_time
            self.robot_command.initialized = True

        self.error_count = [0] * self.n_motor

        self.timer = self.create_timer(1 / P.CAN_HZ, self.timer_callback)

    def timer_callback(self):
        msg = MeviusLog()
        msg.header.stamp = Time()
        jointstate_msg = JointState()
        jointstate_msg.header.stamp = Time()

        with self.robot_command.lock:
            ref_angle = self.robot_command.angle[:]
            ref_velocity = self.robot_command.velocity[:]
            ref_kp = self.robot_command.kp[:]
            ref_kd = self.robot_command.kd[:]
            ref_torque = self.robot_command.torque[:]
        print(ref_angle[:])

        pos_list = [0] * self.n_motor
        vel_list = [0] * self.n_motor
        cur_list = [0] * self.n_motor
        tem_list = [0] * self.n_motor
        for i, motor in enumerate(self.motors):
            try:
                pos, vel, cur, tem = motor.send_rad_command(
                    ref_angle[i], ref_velocity[i], ref_kp[i], ref_kd[i], ref_torque[i]
                )
            except Exception as e:
                self.error_count[i] += 1
                print(
                    "# Can Reciver is Failed for {}, ({})".format(
                        P.JOINT_NAME[i], self.error_count[i]
                    )
                )
                print(e)
                continue
            pos_list[i] = pos
            vel_list[i] = vel
            cur_list[i] = cur
            tem_list[i] = tem

        with self.robot_state.lock:
            self.robot_state.angle = pos_list
            self.robot_state.velocity = vel_list
            self.robot_state.current = cur_list
            self.robot_state.temperature = tem_list

        jointstate_msg.name = P.JOINT_NAME
        jointstate_msg.position = pos_list
        jointstate_msg.velocity = vel_list
        jointstate_msg.effort = cur_list
        self.jointstate_pub.pub.publish(jointstate_msg)

        msg.angle = pos_list
        msg.velocity = vel_list
        msg.current = cur_list
        msg.temperature = tem_list

        with self.peripherals_state.lock:
            msg.body_vel = self.peripherals_state.body_vel[:]
            msg.body_quat = self.peripherals_state.body_quat[:]
            msg.body_gyro = self.peripherals_state.body_gyro[:]
            msg.body_acc = self.peripherals_state.body_acc[:]

        msg.ref_angle = ref_angle
        msg.ref_velocity = ref_velocity
        msg.ref_kp = ref_kp
        msg.ref_kd = ref_kd
        msg.ref_torque = ref_torque

        self.state_pub.pub.publish(msg)


class KeyboardJoy(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripheral_state: PeripheralState,
    ):
        super().__init__("key_joy_node")
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripheral_state = peripheral_state
        self.subscription = self.create_subscription(
            Joy,
            "/joy",
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
            command_callback("STANDBY-STANDUP", self.robot_state, self.robot_command)
        elif two_pushed == 1:
            command_callback("STANDUP-WALK", self.robot_state, self.robot_command)


class MeviusLogPub(Node):
    def __init__(self):
        super().__init__("mevius_log")
        self.pub = self.create_publisher(MeviusLog, "/mevius_log", 2)
        print("Init mevius_log node")


class JointStatePub(Node):
    def __init__(self):
        super().__init__("joint_states")
        self.pub = self.create_publisher(JointState, "/joint_states", 2)
        print("Init joint_states node")


class SimCommunication(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripherals_state: PeripheralState,
    ):
        super().__init__("sim_communication")
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripherals_state = peripherals_state

        print("Init sim node")
        # import tf

        xml_path = os.path.abspath("src/mevius/models/scene.xml")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_step(self.model, self.data)

        self.mujoco_joint_names = [
            self.model.joint(i).name for i in range(self.model.njnt)
        ]
        with self.robot_state.lock:
            for i, name in enumerate(P.JOINT_NAME):
                idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                self.robot_state.angle[i] = self.data.qpos[7 + idx]
                self.robot_state.velocity[i] = self.data.qvel[6 + idx]
                self.robot_state.current[i] = 0.0
                self.robot_state.temperature[i] = 25.0

        for i, name in enumerate(P.JOINT_NAME):
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self.data.ctrl[idx] = P.STANDBY_ANGLE[i]

        self.state_pub = MeviusLogPub()
        self.jointstate_pub = JointStatePub()
        # state_pub = rospy.Publisher('mevius_log', MeviusLog, queue_size=2)
        # jointstate_pub = rospy.Publisher('joint_states', JointState, queue_size=2)

        with self.robot_state.lock:
            self.robot_state.angle = P.STANDBY_ANGLE[:]

        with self.robot_command.lock:
            self.robot_command.command = "STANDBY"
            self.robot_command.angle = P.STANDBY_ANGLE[:]
            self.robot_command.initial_angle = P.STANDBY_ANGLE[:]
            self.robot_command.final_angle = P.STANDBY_ANGLE[:]
            self.robot_command.interpolating_time = 3.0
            self.robot_command.remaining_time = self.robot_command.interpolating_time
            self.robot_command.initialized = True

        self.mujoco_Hz = 1 / 50
        self.timer = self.create_timer(self.mujoco_Hz, self.timer_callback)
        # rate = self.create_rate(200) # mujoco hz

    def timer_callback(self):
        if self.viewer.is_alive:
            pass

        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.viewer.cam.trackbodyid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )

        msg = MeviusLog()

        now = self.get_clock().now()
        builtin_time = Time()
        builtin_time.sec = now.seconds_nanoseconds()[0]  # 現在の秒数
        builtin_time.nanosec = now.seconds_nanoseconds()[1]  # 現在のナノ秒

        jointstate_msg = JointState()
        jointstate_msg.header.stamp = builtin_time

        with self.robot_command.lock:
            ref_angle = self.robot_command.angle[:]
            ref_velocity = self.robot_command.velocity[:]
            ref_kp = self.robot_command.kp[:]
            ref_kd = self.robot_command.kd[:]
            ref_torque = self.robot_command.torque[:]

        for i, name in enumerate(P.JOINT_NAME):  # mevius
            if name in self.mujoco_joint_names:  # mujoco
                idx = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )  # mujoco
                self.data.ctrl[idx] = ref_angle[i]

        mujoco.mj_step(self.model, self.data)

        with self.robot_state.lock:
            for i, name in enumerate(P.JOINT_NAME):
                if name in self.mujoco_joint_names:
                    idx = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                    )
                    self.robot_state.angle[i] = self.data.qpos[7 + idx]
                    self.robot_state.velocity[i] = self.data.qvel[6 + idx]
                    self.robot_state.current[i] = 0.0
                    self.robot_state.temperature[i] = 25.0

        with self.robot_state.lock:
            msg.angle = self.robot_state.angle[:]
            msg.velocity = self.robot_state.velocity[:]
            msg.current = self.robot_state.current[:]
            msg.temperature = self.robot_state.temperature[:]

        jointstate_msg.name = P.JOINT_NAME
        jointstate_msg.position = msg.angle
        jointstate_msg.velocity = msg.velocity
        jointstate_msg.effort = msg.current
        self.jointstate_pub.pub.publish(jointstate_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now()
        base_lin_vel_in_world = np.array(
            [self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]]
        )
        # scipy quaternion: [x, y, z, w]
        base_quat_in_world = np.array(
            [self.data.qpos[4], self.data.qpos[5], self.data.qpos[6], self.data.qpos[3]]
        )
        base_lin_vel_in_base = (
            Rotation.from_quat(base_quat_in_world).inv().apply(base_lin_vel_in_world)
        )
        odom_msg.twist.twist.linear.x = base_lin_vel_in_base[0]
        odom_msg.twist.twist.linear.y = base_lin_vel_in_base[1]
        odom_msg.twist.twist.linear.z = base_lin_vel_in_base[2]
        # CAUTION! mujoco and isaacgym's quat ordre is different
        odom_msg.pose.pose.orientation.w = self.data.qpos[3]
        odom_msg.pose.pose.orientation.x = self.data.qpos[4]
        odom_msg.pose.pose.orientation.y = self.data.qpos[5]
        odom_msg.pose.pose.orientation.z = self.data.qpos[6]
        realsense_vel_callback(odom_msg, self.peripherals_state)

        gyro_msg = Imu()
        gyro_msg.header.stamp = self.get_clock().now()
        # for realsense
        gyro_msg.angular_velocity.x = self.data.qvel[4]
        gyro_msg.angular_velocity.y = self.data.qvel[5]
        gyro_msg.angular_velocity.z = self.data.qvel[3]
        realsense_gyro_callback(gyro_msg, self.peripherals_state)

        acc_msg = Imu()
        acc_msg.header.stamp = self.get_clock().now()
        # for realsense
        acc_msg.linear_acceleration.x = self.data.qacc[1]
        acc_msg.linear_acceleration.y = self.data.qacc[2]
        acc_msg.linear_acceleration.z = self.data.qacc[0]
        realsense_acc_callback(acc_msg, self.peripherals_state)

        with self.peripherals_state.lock:
            msg.body_vel = self.peripherals_state.body_vel[:]
            msg.body_quat = self.peripherals_state.body_quat[:]
            msg.body_gyro = self.peripherals_state.body_gyro[:]
            msg.body_acc = self.peripherals_state.body_acc[:]

        msg.ref_angle = ref_angle
        msg.ref_velocity = ref_velocity
        msg.ref_kp = ref_kp
        msg.ref_kd = ref_kd
        msg.ref_torque = ref_torque

        self.state_pub.pub.publish(msg)

        self.viewer.render()


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
