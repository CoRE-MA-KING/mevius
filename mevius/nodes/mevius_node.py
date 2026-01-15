"""MEVIUS base node."""

from rclpy.node import Node


class Mevius(Node):
    def __init__(self):
        super().__init__("mevius")
        print("Init Mevius Node")
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        pass
        # print('mevius callback!')
