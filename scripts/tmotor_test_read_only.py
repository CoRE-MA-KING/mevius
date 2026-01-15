import argparse
import select
import sys
import time

import numpy as np
from tmotor_lib import CanMotorController


def setZeroPosition(motor):
    pos, _, _, _ = motor.set_zero_position()
    # motor.set_zero_position()
    # while abs(np.rad2deg(pos)) > 0.5:
    #     pos, vel, cur = motor.set_zero_position()
    #     print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), cur))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--device", "-d", type=str, default="can0", help="can interface name"
    )
    parser.add_argument(
        "--ids", "-i", type=int, nargs="+", default=None, help="motor ids to control"
    )
    parser.add_argument(
        "--kp", type=float, default=30.0, help="p gain for position control"
    )
    parser.add_argument(
        "--kd", type=float, default=1.0, help="d gain for position control"
    )
    args = parser.parse_args()

    print("# using Socket {} for can communication".format(args.device))
    print("# motor ids: {}".format(args.ids))
    assert args.ids is not None, "please input motor ids"

    ids = args.ids
    motors = {}
    for id in ids:
        motor_dir = 1
        motors[id] = CanMotorController(args.device, id, motor_dir, "AK70_10_V1p1")

    print("Enabling Motors..")
    for motor_id, motor_controller in motors.items():
        pos, vel, cur, tem = motor_controller.enable_motor()
        print(
            "Motor {} Status: Pos: {}, Vel: {}, Torque: {}, Temp: {}".format(
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

    # for deg in np.linspace(0.0, 360.0, 36):
    while True:
        for motor_id, motor_controller in motors.items():
            deg = 0
            pos, vel, cur, tem = motor_controller.send_deg_command(
                deg, 0, args.kp, args.kd, 0
            )
            print(deg)
            print(
                "Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(
                    motor_id, pos, vel, cur, tem
                )
            )
            time.sleep(0.1)

    time.sleep(1)

    print("Disabling Motors...")
    for motor_id, motor_controller in motors.items():
        pos, vel, cur, tem = motor_controller.disable_motor()
        time.sleep(0.2)
        print(
            "Motor {} Status: Pos: {}, Vel: {}, Torque: {}".format(
                motor_id, pos, vel, cur
            )
        )


if __name__ == "__main__":
    main()
