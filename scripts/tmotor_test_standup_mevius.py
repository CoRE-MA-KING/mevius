import argparse
import math
import time

import numpy as np
from tmotor_lib import CanMotorController

start_angles_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

end_angles_deg = [
    10.0,
    -40.0,
    90.0,
    -10.0,
    -40.0,
    90.0,
    10.0,
    -40.0,
    90.0,
    -10.0,
    -40.0,
    90.0,
]

step_num = 100


def setZeroPosition(motor):
    pos, _, _, _ = motor.set_zero_position()
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, cur, _ = motor.set_zero_position()
        print(
            'Position: {}, Velocity: {}, Torque: {}'.format(np.rad2deg(pos), np.rad2deg(vel), cur)
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', '-d', type=str, default='can0', help='can interface name')
    parser.add_argument('--kp', type=float, default=200.0, help='p gain for position control')
    parser.add_argument('--kd', type=float, default=1.0, help='d gain for position control')
    args = parser.parse_args()

    print('# using Socket {} for can communication'.format(args.device))
    # print("# motor ids: {}".format(args.ids))
    # assert args.ids is not None, "please input motor ids"
    # ids = [1,2,3]
    ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    motors = {}
    for motor_id in ids:
        motor_dir = 1
        if motor_id in [1, 4, 5, 6, 11, 12]:
            motor_dir = -1
        motors[motor_id] = CanMotorController(args.device, motor_id, motor_dir, 'AK70_10_V1p1')

    print('Enabling Motors..')
    for motor_id, motor_controller in motors.items():
        pos, vel, cur, tem = motor_controller.enable_motor()
        print(
            'Motor {} Status: Pos: {}, Vel: {}, Torque: {}, Temp: {}'.format(
                motor_id, pos, vel, cur, tem
            )
        )

    time.sleep(1)

    pos_vec = []
    vel_vec = []
    cur_vec = []
    tem_vec = []
    for motor_id, motor_controller in motors.items():
        setZeroPosition(motor_controller)
        pos, vel, cur, tem = motor_controller.send_deg_command(0, 0, 0.0, 0.0, 0)
        pos_vec.append(pos)
        vel_vec.append(vel)
        cur_vec.append(cur)
        tem_vec.append(tem)

    for cur_step in range(step_num):
        for motor_id, motor_controller in motors.items():
            tgt_deg = start_angles_deg[motor_id - 1] + (
                end_angles_deg[motor_id - 1] - start_angles_deg[motor_id - 1]
            ) * (cur_step / step_num)
            pos, vel, cur, tem = motor_controller.send_deg_command(tgt_deg, 0, args.kp, args.kd, 0)
            print(tgt_deg)
            print(
                'Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}'.format(
                    motor_id, pos, vel, cur, tem
                )
            )
        time.sleep(0.02)

    cur_time_s = 0.0
    period_s = 3.0
    omega = 2 * math.pi / period_s
    cyclerate_hz = 100.0
    sinamp_deg = 0.0
    while True:
        if cur_time_s > (period_s):
            cur_time_s = 0.0
        else:
            cur_time_s = cur_time_s + (1 / cyclerate_hz)
        offset_angle = (-1 + math.cos(omega * cur_time_s)) * sinamp_deg
        for motor_id, motor_controller in motors.items():
            if motor_id in [1, 4, 7, 10]:
                motor_offset_deg = 0
            elif motor_id in [2, 5, 8, 11]:
                motor_offset_deg = -offset_angle
                # if motor_id in [2,5]:
                #    motor_offset_deg = - offset_angle
                # else:
                #    motor_offset_deg = offset_angle
            elif motor_id in [3, 6, 9, 12]:
                motor_offset_deg = 2 * offset_angle
                # if motor_id in [3,6]:
                #    motor_offset_deg = -2* offset_angle
                # else:
                #    motor_offset_deg = 2*offset_angle

            tgt_deg = (
                start_angles_deg[motor_id - 1]
                + (end_angles_deg[motor_id - 1] - start_angles_deg[motor_id - 1])
                * (cur_step / step_num)
                + motor_offset_deg
            )
            pos, vel, cur, tem = motor_controller.send_deg_command(tgt_deg, 0, args.kp, args.kd, 0)
            print(tgt_deg)
            print(
                'Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}'.format(
                    motor_id, pos, vel, cur, tem
                )
            )
        time.sleep(1.0 / cyclerate_hz)
    # time.sleep(1)

    # print("Disabling Motors...")
    # for motor_id, motor_controller in motors.items():
    #     pos, vel, cur, tem = motor_controller.disable_motor()
    #     time.sleep(0.2)
    #     print("Motor {} Status: Pos: {}, Vel: {}, Torque: {}".format(motor_id, pos, vel, cur))


if __name__ == '__main__':
    main()
