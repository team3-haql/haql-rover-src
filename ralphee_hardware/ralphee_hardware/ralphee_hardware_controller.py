import rclpy
import asyncio
from rclpy.node import Node
from ralphee_motor_interface import init_motors, update_motors
from ralphee_servo_interface import init_servos, update_servos

# Twist is deprecated! May need to use TwistStamped
from geometry_msgs.msg import Twist


class RalpheeHardwareController(Node):
    """
        Class that subscribes to 'cmd_vel' topic, retreives angle and velocity from twist object, and sends it to hardware.
    """
    def __init__(self):
        """ 
            Constructs RalpheeHardwareController class
        """
        super().__init__('ralphee_hardware_controller')
        self.publisher_ = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

        self.get_logger().info(f'Initializing Hardware!')

        loop = asyncio.get_event_loop() # Runs async function in non async function!

        tasks = init_motors(), init_servos()
        self.motors, self.arduino = loop.run_until_complete(asyncio.gather(*tasks))

        loop.close()

        self.get_logger().info(f'Initialized Hardware!')

    def listener_callback(self, msg: Twist):
        """
            Receives velocity and angle and sends it to hardware!
            Args:
                msg:
                    Twist object that comes with angle and velocity.
        """
        velocity: float = msg.linear.x
        angle: float = msg.angular.z
        self.get_logger().info(f'Velocity: {velocity}, Angle: {angle}')

        self.get_logger().info(f'Updating motors and servos!')

        loop = asyncio.get_event_loop() # Runs async function in non async function!

        tasks = update_motors(velocity, angle, self.motors), update_servos(angle, self.motors)
        self.motors, self.arduino = loop.run_until_complete(asyncio.gather(*tasks))

        loop.close()

        self.get_logger().info(f'Updated motors and servos!')

def main(args=None):
    """
        Spins up server
    """
    rclpy.init(args=args)

    ralphee_hardware_controller = RalpheeHardwareController()

    rclpy.spin(ralphee_hardware_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ralphee_hardware_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()