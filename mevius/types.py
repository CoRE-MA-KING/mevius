"""Data types for MEVIUS."""

import threading
from typing import Literal


class RobotState:
    """Represents the state of the robot."""

    def __init__(self, n_motor=12):
        """Initializes the robot state."""
        self.angle = [0.0] * n_motor
        self.velocity = [0.0] * n_motor
        self.current = [0.0] * n_motor
        self.temperature = [0.0] * n_motor
        self.lock = threading.Lock()


class PeripheralState:
    """Represents the state of the robot's peripherals."""

    def __init__(self):
        """Initializes the peripheral state."""
        self.realsense_last_time = None
        self.body_vel = [0.0] * 3
        self.body_quat = [0.0] * 4
        self.body_gyro = [0.0] * 3
        self.body_acc = [0.0] * 3
        self.spacenav_enable = False
        self.spacenav = [0.0] * 8
        self.virtual_enable = False
        self.virtual = [0.0] * 5
        self.lock = threading.Lock()


ModeCommand = Literal['STANDBY', 'STANDBY-STANDUP', 'STANDUP', 'STANDUP-WALK', 'WALK', 'DEBUG']


class RobotCommand:
    """Represents a command to the robot."""

    def __init__(self, n_motor=12, joint_name=[], stiffness={}, damping={}):
        """Initializes the robot command."""
        self.angle = [0.0] * n_motor
        self.velocity = [0.0] * n_motor
        self.kp = []
        self.kd = []
        self.coef = 1.0
        for name in joint_name:
            for key in stiffness.keys():
                if key in name:
                    self.kp.append(stiffness[key] * self.coef)
                    self.kd.append(damping[key] * self.coef)
        assert len(self.kp) == n_motor
        assert len(self.kd) == n_motor
        self.torque = [0.0] * n_motor

        self.command = 'STANDBY'
        self.initial_angle = [0.0] * n_motor
        self.final_angle = [0.0] * n_motor
        self.interpolating_time = 0.0
        self.remaining_time = 0.0
        self.initialized = False

        self.lock = threading.Lock()
