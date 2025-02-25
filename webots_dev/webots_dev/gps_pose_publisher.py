import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

import numpy as np

class GpsPoseNode(Node):
    def __init__(self):
        super().__init__('gps_pose_node')

        # declare parameters
        self.declare_parameter('update_rate', 20)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        update_rate = self.get_parameter('update_rate').value

        # setup subscriptions
        self.subscription = self.create_subscription(
            PointStamped,
            '/gps_point',
            self.gps_callback,
            10)
        self.subscription = self.create_subscription(
            Imu,
            '/imu_no_cov',
            self.imu_callback,
            10)

        # setup broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = TransformStamped()

        # create timer
        self.get_logger().info(f'Update rate: {update_rate}')
        self.create_timer(1.0 / update_rate, self.timer_callback)

    def gps_callback(self, msg):
        self.transform.transform.translation.x = msg.point.x
        self.transform.transform.translation.y = msg.point.y
        self.transform.transform.translation.z = msg.point.z

    def imu_callback(self, msg):
        self.transform.transform.rotation = msg.orientation

    def timer_callback(self):
        # odom -> base_link
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = self.get_parameter('odom_frame').value
        self.transform.child_frame_id = self.get_parameter('base_frame').value
        self.tf_broadcaster.sendTransform(self.transform)

        # map -> odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('world_frame').value
        t.child_frame_id = self.get_parameter('odom_frame').value
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    gps_pose_node= GpsPoseNode()
    rclpy.spin(gps_pose_node)
    gps_pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
