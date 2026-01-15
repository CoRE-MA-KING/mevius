"""SimCommunication node for MEVIUS."""

import os

import mujoco
import mujoco_viewer
import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState

from ..callbacks import (
    realsense_acc_callback,
    realsense_gyro_callback,
    realsense_vel_callback,
)
from ..types import PeripheralState, RobotCommand, RobotState
from .publisher import JointStatePub, MeviusLogPub


class SimCommunication(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripherals_state: PeripheralState,
    ):
        super().__init__('sim_communication')
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripherals_state = peripherals_state

        print('Init sim node')
        # import tf

        self.declare_parameter('JOINT_NAME', [''])
        self.declare_parameter('STANDBY_ANGLE', [0.0] * 12)

        self.joint_name = self.get_parameter('JOINT_NAME').get_parameter_value().string_array_value
        self.standby_angle = (
            self.get_parameter('STANDBY_ANGLE').get_parameter_value().double_array_value
        )

        xml_path = os.path.abspath('src/mevius/models/scene.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_step(self.model, self.data)

        self.mujoco_joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        with self.robot_state.lock:
            for i, name in enumerate(self.joint_name):
                idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                self.robot_state.angle[i] = self.data.qpos[7 + idx]
                self.robot_state.velocity[i] = self.data.qvel[6 + idx]
                self.robot_state.current[i] = 0.0
                self.robot_state.temperature[i] = 25.0

        for i, name in enumerate(self.joint_name):
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self.data.ctrl[idx] = self.standby_angle[i]

        self.state_pub = MeviusLogPub()
        self.jointstate_pub = JointStatePub()
        # state_pub = rospy.Publisher('mevius_log', MeviusLog, queue_size=2)
        # jointstate_pub = rospy.Publisher('joint_states', JointState, queue_size=2)

        with self.robot_state.lock:
            self.robot_state.angle = self.standby_angle

        with self.robot_command.lock:
            self.robot_command.command = 'STANDBY'
            self.robot_command.angle = self.standby_angle
            self.robot_command.initial_angle = self.standby_angle
            self.robot_command.final_angle = self.standby_angle
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
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link'
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

        for i, name in enumerate(self.joint_name):  # mevius
            if name in self.mujoco_joint_names:  # mujoco
                idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)  # mujoco
                self.data.ctrl[idx] = ref_angle[i]

        mujoco.mj_step(self.model, self.data)

        with self.robot_state.lock:
            for i, name in enumerate(self.joint_name):
                if name in self.mujoco_joint_names:
                    idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                    self.robot_state.angle[i] = self.data.qpos[7 + idx]
                    self.robot_state.velocity[i] = self.data.qvel[6 + idx]
                    self.robot_state.current[i] = 0.0
                    self.robot_state.temperature[i] = 25.0

        with self.robot_state.lock:
            msg.angle = self.robot_state.angle[:]
            msg.velocity = self.robot_state.velocity[:]
            msg.current = self.robot_state.current[:]
            msg.temperature = self.robot_state.temperature[:]

        jointstate_msg.name = self.joint_name
        jointstate_msg.position = msg.angle
        jointstate_msg.velocity = msg.velocity
        jointstate_msg.effort = msg.current
        self.jointstate_pub.pub.publish(jointstate_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now()
        base_lin_vel_in_world = np.array([self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]])
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
