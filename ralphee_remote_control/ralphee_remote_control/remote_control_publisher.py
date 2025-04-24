import rclpy
from rclpy.node import Node
from ralphee_remote_control.remote_controller import Controller

from std_msgs.msg import Float64MultiArray, Int16


class ControllerPublisher(Node):

    def __init__(self):
        super().__init__('controller_publisher')
        self.cmd_vel_pub = self.create_publisher(Float64MultiArray, 'angle_vel_controller', 10)
        self.state_pub = self.create_publisher(Int16, 'state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.controller = Controller()

    def timer_callback(self):
        vel_angle_msg = Float64MultiArray()
        state_msg = Int16()

        vel = self.controller.velocity
        angle = self.controller.angle

        vel_angle_msg.data.append(vel)
        vel_angle_msg.data.append(angle)

        x = self.controller.X # auto
        y = self.controller.Y # controller

        if x == 1:
            state_msg.data = 1 # State change to controller
            self.state_pub.publish(state_msg)
        if y == 1:
            state_msg.data = 2 # State change to nav2
            self.state_pub.publish(state_msg)

        self.cmd_vel_pub.publish(vel_angle_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ControllerPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()