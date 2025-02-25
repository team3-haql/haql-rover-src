
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CalibrateStraight(Node):

    def __init__(self):
        super().__init__('calibrate_straight')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.distance_traveled = 0
        self.twist = Twist()
        self.twist.linear.x = 0.2

    def timer_callback(self):
        current_time = self.get_clock().now()
        time_elapsed = current_time - self.start_time
        self.distance_traveled = self.twist.linear.x * time_elapsed.nanoseconds / 1e9
        if self.distance_traveled >= 3.0:
            self.twist.linear.x = 0.0
            self.get_logger().info('Stopping')
            self.timer.cancel()
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    calibrate_straight = CalibrateStraight()
    rclpy.spin(calibrate_straight)
    calibrate_straight.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
