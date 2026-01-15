"""MEVIUS base node."""

from rclpy.node import Node


class Mevius(Node):
    def __init__(self):
        super().__init__("mevius")
        print("Init Mevius Node")

        self.declare_parameter("JOINT_NAME", [""])
        self.declare_parameter("control.stiffness.collar", 0.0)
        self.declare_parameter("control.stiffness.hip", 0.0)
        self.declare_parameter("control.stiffness.knee", 0.0)
        self.declare_parameter("control.damping.collar", 0.0)
        self.declare_parameter("control.damping.hip", 0.0)
        self.declare_parameter("control.damping.knee", 0.0)

        self.joint_names = (
            self.get_parameter("JOINT_NAME").get_parameter_value().string_array_value
        )
        self.stiffness = {
            "collar": self.get_parameter("control.stiffness.collar")
            .get_parameter_value()
            .double_value,
            "hip": self.get_parameter("control.stiffness.hip")
            .get_parameter_value()
            .double_value,
            "knee": self.get_parameter("control.stiffness.knee")
            .get_parameter_value()
            .double_value,
        }
        self.damping = {
            "collar": self.get_parameter("control.damping.collar")
            .get_parameter_value()
            .double_value,
            "hip": self.get_parameter("control.damping.hip")
            .get_parameter_value()
            .double_value,
            "knee": self.get_parameter("control.damping.knee")
            .get_parameter_value()
            .double_value,
        }

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        pass
        # print('mevius callback!')
