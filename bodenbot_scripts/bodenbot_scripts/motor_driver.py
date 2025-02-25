
import rclpy
import numpy as np
from . import motor_controller

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # declare wheels
        self.declare_parameter('left_wheels', ['none'])
        self.left_wheels = []
        for wheel in self.get_parameter('left_wheels').value:
            self.declare_parameter(wheel + '.id', 0)
            self.declare_parameter(wheel + '.reversed', False)
            info = self.get_parameters_by_prefix(wheel)
            self.left_wheels.append({
                'wheel': wheel,
                'id': info['id'].value,
                'reversed': info['reversed'].value,
                'pub': self.create_publisher(Float32, f'/wheel/{wheel}/velocity', 10),
            })
            self.get_logger().info(f'{wheel}: id = {info["id"].value}, reversed = {info["reversed"].value}')

        self.declare_parameter('right_wheels', ['none'])
        self.right_wheels = []
        for wheel in self.get_parameter('right_wheels').value:
            self.declare_parameter(wheel + '.id', 0)
            self.declare_parameter(wheel + '.reversed', False)
            info = self.get_parameters_by_prefix(wheel)
            self.right_wheels.append({
                'wheel': wheel,
                'id': info['id'].value,
                'reversed': info['reversed'].value,
                'pub': self.create_publisher(Float32, f'/wheel/{wheel}/velocity', 10),
            })
            self.get_logger().info(f'{wheel}: id = {info["id"].value}, reversed = {info["reversed"].value}')

        # body sizes
        self.declare_parameter('wheel_separation', 0.0)
        self.declare_parameter('wheel_radius', 0.0)

        wheel_separation = self.get_parameter('wheel_separation').value
        wheel_radius = self.get_parameter('wheel_radius').value
        self.get_logger().info(f'wheel_separation = {wheel_separation}, wheel_radius = {wheel_radius}')

        # log velocities
        self.declare_parameter('update_rate', 1)
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(1 / update_rate, self.log_velocities)

    def cmd_vel_callback(self, msg):
        wheel_separation = self.get_parameter('wheel_separation').value
        wheel_radius = self.get_parameter('wheel_radius').value

        vel = msg.linear.x
        ang = msg.angular.z

        left_vel = vel - ang * wheel_separation / 2
        right_vel = vel + ang * wheel_separation / 2

        left_w = left_vel / wheel_radius
        right_w = right_vel / wheel_radius

        for wheel in self.left_wheels:
            v = -left_w if wheel['reversed'] else left_w
            motor_controller.write_motor_value(wheel['id'], v)

        for wheel in self.right_wheels:
            v = -right_w if wheel['reversed'] else right_w
            motor_controller.write_motor_value(wheel['id'], v)

    def log_velocities(self):
        for wheel in self.left_wheels + self.right_wheels:
            value = motor_controller.read_motor_value(wheel['id'])
            if wheel['reversed']:
                value = -value
            wheel['pub'].publish(Float32(data=value))


def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriver()
    rclpy.spin(motor_driver_node)
    motor_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
