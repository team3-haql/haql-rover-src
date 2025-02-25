import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

import numpy as np

class GpsOdomNode(Node):
    def __init__(self):
        super().__init__('gps_odom_node')
        self.subscription = self.create_subscription(
            PointStamped,
            '/gps_point',
            self.gps_point_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'gps', 10)

    def gps_point_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "map"
        odom.child_frame_id = "gps"
        odom.pose.pose.position = msg.point

        odom.pose.covariance= np.diag([0.05, 0.05, 0.05, 0, 0, 0]).reshape(-1)

        # Publish the odom data
        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    gps_odom_node = GpsOdomNode()
    rclpy.spin(gps_odom_node)
    gps_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
