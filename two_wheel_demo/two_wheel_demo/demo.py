import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class ODriveDemoNode(Node):

    def __init__(self):
        super().__init__('odrive_demo_publisher')
        self.axis0_publisher = self.create_publisher(Float32, 'axis0_vel_sub', 10)
        self.axis1_publisher = self.create_publisher(Float32, 'axis1_vel_sub', 10)
        
        timer_period = 1 / 50
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def __del__(self):
        msg = Float32()
        msg.data = 0.0
        self.axis0_publisher.publish(msg)
        self.axis1_publisher.publish(msg)

    def timer_callback(self):
        msg = Float32()
        msg.data = -math.cos(self.i)
        self.axis0_publisher.publish(msg)

        msg.data = math.sin(self.i)
        self.axis1_publisher.publish(msg)

        self.i += (math.pi / 100)


def main(args=None):
    rclpy.init(args=args)

    odrive_demo_publisher = ODriveDemoNode()

    rclpy.spin(odrive_demo_publisher)
    odrive_demo_publisher.destroy_node()
    rclpy.shutdown()

