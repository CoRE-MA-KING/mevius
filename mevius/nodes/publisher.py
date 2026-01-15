"""Publisher nodes for MEVIUS."""

from mevius_massage.msg._mevius_log import MeviusLog
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
