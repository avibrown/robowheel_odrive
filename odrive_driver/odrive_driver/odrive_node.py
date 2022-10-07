import rclpy
from rclpy.node import Node
from odrive.enums import *
from std_msgs.msg import Float32
from .odrive_command import ODriveController


class ODriveNode(Node):

    def __init__(self, odrv0):
        super().__init__('driver')
        self.odrv0 = odrv0

        self.axis0_vel_pub = self.create_publisher(
            Float32, 'axis0_vel_pub', 50)
        self.axis1_vel_pub = self.create_publisher(
            Float32, 'axis1_vel_pub', 50)

        self.axis0_pos_pub = self.create_publisher(
            Float32, 'axis0_pos_pub', 50)
        self.axis1_pos_pub = self.create_publisher(
            Float32, 'axis1_pos_pub', 50)

        timer_period = 1 / 100
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.axis0_vel_sub = self.create_subscription(
            Float32, 'axis0_vel_sub', self.axis0_vel_callback, 50)
        self.axis1_vel_sub = self.create_subscription(
            Float32, 'axis1_vel_sub', self.axis1_vel_callback, 50)

        self.axis0_pos_sub = self.create_subscription(
            Float32, 'axis0_pos_sub', self.axis0_pos_callback, 50)
        self.axis1_pos_sub = self.create_subscription(
            Float32, 'axis1_pos_sub', self.axis1_pos_callback, 50)

    def timer_callback(self):
        msg = Float32()
        msg.data = self.odrv0.get_velocity(0)
        self.axis0_vel_pub.publish(msg)

        msg.data = self.odrv0.get_velocity(1)
        self.axis1_vel_pub.publish(msg)

        msg.data = self.odrv0.get_position(0)
        self.axis0_pos_pub.publish(msg)

        msg.data = self.odrv0.get_position(1)
        self.axis1_pos_pub.publish(msg)

    def axis0_vel_callback(self, msg):
        print(f'axis0 vel: {msg}')
        self.odrv0.command_velocity(0, msg.data)

    def axis1_vel_callback(self, msg):
        self.get_logger().info(f'axis1 vel: {msg}')
        self.odrv0.command_velocity(1, msg.data)

    def axis0_pos_callback(self, msg):
        self.get_logger().info(f'axis0 pos: {msg}')
        self.odrv0.command_position(0, msg.data)

    def axis1_pos_callback(self, msg):
        self.get_logger().info(f'axis1 pos: {msg}')
        self.odrv0.command_velocity(1, msg.data)


def main(args=None):
    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()
