import argparse
import sys
import select
import time
import numpy as np
from tmotor_lib import CanMotorController

start_angles_deg = [0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0]

end_angles_deg = [30.0, 30.0, 30.0,
                  30.0, 30.0, 30.0,
                  30.0, 30.0, 30.0,
                  30.0, 30.0, 30.0]

step_num = 30

def setZeroPosition(motor):
    pos, _, _ = motor.set_zero_position()
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, cur = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), cur))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", '-d', type=str, default="can0", help="can interface name")
    parser.add_argument("--kp", type=float, default=3.0, help="p gain for position control")
    parser.add_argument("--kd", type=float, default=1.0, help="d gain for position control")
    args = parser.parse_args()

    print("# using Socket {} for can communication".format(args.device))
    print("# motor ids: {}".format(args.ids))
    assert args.ids is not None, "please input motor ids"

    ids = [1,2,3,4,5,6,7,8,9,10,11,12]
    motors = {}
    for id in ids:
        motor_dir = 1
        if id in [1, 4, 5, 6, 11, 12]:
            motor_dir = -1
            motors[id] = CanMotorController(args.device, id, motor_dir=motor_dir,"AK70_10_V1p1")

    print("Enabling Motors..")
    for motor_id, motor_controller in motors.items():
        pos, vel, cur, tem = motor_controller.enable_motor()
        print("Motor {} Status: Pos: {}, Vel: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, cur, tem))

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
            tgt_deg = start_angles_deg[motors.can_id-1] + (end_angles_deg[motors.can_id-1] - start_angles_deg[motors.can_id-1])*(cur_step/step_num)
            pos, vel, cur, tem = motor_controller.send_deg_command(deg, 0, args.kp, args.kd, 0)
            print(deg)
            print("Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, cur, tem))
        time.sleep(0.1)

    # time.sleep(1)

    # print("Disabling Motors...")
    # for motor_id, motor_controller in motors.items():
    #     pos, vel, cur, tem = motor_controller.disable_motor()
    #     time.sleep(0.2)
    #     print("Motor {} Status: Pos: {}, Vel: {}, Torque: {}".format(motor_id, pos, vel, cur))


if __name__ == "__main__":
    main()
