
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import math

def main(args=None):
    rclpy.init(args=args)
    node = Node('calibrate_box')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    for n in range(5):
        node.get_logger().info(f'starting box {n}')
        for i in range(4):
            # drive foward 2 meters over 10 seconds
            for _ in range(100):
                twist = Twist()
                twist.linear.x = 0.2
                publisher.publish(twist)
                node.get_clock().sleep_for(Duration(seconds=0.1))

            node.get_logger().info(f'Turn #{i}')

            # turn 90 degrees over 8 seconds
            for _ in range(80):
                twist = Twist()
                twist.angular.z = math.pi / 2 / 8
                publisher.publish(twist)
                node.get_clock().sleep_for(Duration(seconds=0.1))

    node.get_logger().info('Stopping')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
