from px4_msgs.srv import VehicleCommand
from px4_msgs.msg import ActuatorMotors

import rclpy
from rclpy.node import Node

TOPIC = '/something'

class Px4Advertiser(Node):
    def __init__(self):
        super().__init__('px4_advertiser')
        self.pub = self.create_publisher(ActuatorMotors, TOPIC, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        msg = ActuatorMotors()
        msg.timestamp = 0
        msg.timestamp_sample = 0
        msg.reversible_flags = 0
        # 1 is max positive speed
        # -1 is max negative speed
        msg.control = [0.1, -0.1, 
                       0.0, 0.0,
                       0.0, 0.0, 
                       0.0, 0.0,
                       0.0, 0.0, 
                       0.0, 0.0]
        self.pub.publish(msg)
        self.get_logger().info('publish!')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Px4Advertiser()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()