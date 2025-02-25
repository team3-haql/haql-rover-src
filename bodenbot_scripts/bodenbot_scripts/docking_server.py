import time

import rclpy
from rclpy.action import ActionServer
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import Twist

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

from boden_interfaces.action import Dock

class DockActionServer(Node):
    def __init__(self):
        super().__init__('dock_action_server')

        self.target_frame = self.declare_parameter(
          'target_frame', 'charger').get_parameter_value().string_value
        self.base_frame = self.declare_parameter(
          'base_frame', 'base_link').get_parameter_value().string_value
        self.threshold_dist = self.declare_parameter(
          'threshold_dist', 0.4).get_parameter_value().double_value
        self.threshold_angle= self.declare_parameter(
          'threshold_angle', 0.2).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(
            self,
            Dock,
            'dock',
            self.execute_callback)

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.goal_handle = goal_handle

        from_frame_rel = self.target_frame
        to_frame_rel = self.base_frame

        status = Dock.Feedback()
        rate = self.create_rate(10)
        while True:
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
                t_rev = self.tf_buffer.lookup_transform(
                    from_frame_rel,
                    to_frame_rel,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                goal_handle.abort()
                result = Dock.Result()
                return result
            
            # calculate values
            dist = np.linalg.norm([t.transform.translation.x, t.transform.translation.y])
            status.distance = dist

            angle = np.arctan2(t.transform.translation.y, t.transform.translation.x)
            status.angle = angle

            alignment = np.arctan2(t_rev.transform.translation.x, t_rev.transform.translation.z)
            status.alignment = alignment

            goal_handle.publish_feedback(status)

            if dist < self.threshold_dist:
                break

            # set movement commands
            velocity = dist * 0.15
            angular = angle * 1.2 # + alignment * 2

            cmd_msg = Twist()
            cmd_msg.linear.x = float(velocity)
            cmd_msg.angular.z = float(angular)
            self.cmd_publisher.publish(cmd_msg)

            rate.sleep()

        self.get_logger().info('Docking Complete')

        goal_handle.succeed()
        result = Dock.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(DockActionServer())
    executor.spin()

    executor.shutdown()


if __name__ == '__main__':
    main()