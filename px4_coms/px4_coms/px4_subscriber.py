from px4_msgs.msg import VehicleOdometry

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

class Px4Subscriber(Node):
    def __init__(self):
        qos_profile = QoSProfile()
        self.sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
    def odometry_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    sub = Px4Subscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()