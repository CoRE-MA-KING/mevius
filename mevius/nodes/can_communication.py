"""CAN communication node for MEVIUS."""

from builtin_interfaces.msg import Time
from mevius_massage.msg._mevius_log import MeviusLog
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .publisher import JointStatePub, MeviusLogPub
from ..tmotor_lib import CanMotorController
from ..types import PeripheralState, RobotCommand, RobotState


class CanCommunication(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripherals_state: PeripheralState,
    ):
        super().__init__('can_communication')
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripherals_state = peripherals_state

        print('Init can node')

        self.declare_parameter('CAN_ID', [0] * 12)
        self.declare_parameter('MOTOR_DIR', [0] * 12)
        self.declare_parameter('JOINT_NAME', [''])
        self.declare_parameter('STANDBY_ANGLE', [0.0] * 12)
        self.declare_parameter('CAN_HZ', 50)

        self.can_id = self.get_parameter('CAN_ID').get_parameter_value().integer_array_value
        self.motor_dir = self.get_parameter('MOTOR_DIR').get_parameter_value().integer_array_value
        self.joint_name = self.get_parameter('JOINT_NAME').get_parameter_value().string_array_value
        self.standby_angle = (
            self.get_parameter('STANDBY_ANGLE').get_parameter_value().double_array_value
        )
        self.can_hz = self.get_parameter('CAN_HZ').get_parameter_value().integer_value

        self.device = 'can0'
        self.motor_type = 'AK70_10_V1p1'
        self.n_motor = 12
        self.motors = [
            CanMotorController(
                self.device,
                self.can_id[i],
                motor_type=self.motor_type,
                motor_dir=self.motor_dir[i],
            )
            for i in range(self.n_motor)
        ]

        print('Enabling Motors...')
        for i, motor in enumerate(self.motors):
            pos, vel, cur, tem = motor.enable_motor()
            print(
                (
                    'Enabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, '
                    'Cur: {:.3f}, Temp: {:.3f}'
                ).format(self.joint_name[i], pos, vel, cur, tem)
            )
            with self.robot_state.lock:
                self.robot_state.angle[i] = pos
                self.robot_state.velocity[i] = vel
                self.robot_state.current[i] = cur
                self.robot_state.temperature[i] = tem
        print('Finish enabling motors!')
        self.state_pub = MeviusLogPub()
        self.jointstate_pub = JointStatePub()
        # state_pub = rospy.Publisher('mevius_log', MeviusLog, queue_size=2)
        # jointstate_pub = rospy.Publisher('joint_states', JointState, queue_size=2)

        print('Setting Initial Offset...')
        for i, motor in enumerate(self.motors):
            motor.set_angle_offset(self.standby_angle[i], deg=False)
            # motor.set_angle_range(joint_params[i][0], joint_params[i][1], deg=False)

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

        self.error_count = [0] * self.n_motor

        self.timer = self.create_timer(1 / self.can_hz, self.timer_callback)

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
                    '# Can Reciver is Failed for {}, ({})'.format(
                        self.joint_name[i], self.error_count[i]
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

        jointstate_msg.name = self.joint_name
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


def main():
    import sys
    import rclpy
    rclpy.init()
    try:
        # Create dummy instances for robot_state, robot_command, peripheral_state
        # These will ideally be replaced by actual data passed via ROS 2 topics/services
        # For now, we need to pass some instances to CanCommunication's constructor
        from ..types import RobotState, RobotCommand, PeripheralState
        robot_state = RobotState()
        robot_command = RobotCommand(joint_name=[], stiffness={}, damping={})  # Placeholder
        peripheral_state = PeripheralState()

        can_node = CanCommunication(robot_state, robot_command, peripheral_state)
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
