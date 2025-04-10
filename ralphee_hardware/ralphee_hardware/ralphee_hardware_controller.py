import rclpy
import math
import asyncio
from rclpy.node import Node
from ralphee_motor_interface import init_motors, update_motors
from ralphee_servo_interface import init_servos, update_servos

from geometry_msgs.msg import Twist

WHEEL_BASE = 1.0

# https://gist.github.com/hdh7485/f87b67b237ef57e46fe77962e343c28b
def convert_trans_rot_vel_to_radius_and_steering_angle(
    velocity: float, angular_velocity: float, wheelbase: float
) -> tuple[float, float]:
    """ 
        Converts velocity and angular velocity into radius of turn and the steering angle of the turn.
        Args:
            velocity:
                x component of cmd_vel.linear
            angular_velocity:
                z comoponent of cmd_vel.angular
            wheelbase:
                TODO: Figure out what wheelbase is
        Returns:
            (radius, steering_angle) 
    """

    if angular_velocity == 0 or velocity == 0:
        return 0

    radius = velocity / angular_velocity
    return radius, math.atan(wheelbase / radius)

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
        angular_velocity: float = msg.angular.z
        radius, steering_angle = convert_trans_rot_vel_to_radius_and_steering_angle(velocity, angular_velocity, WHEEL_BASE)

        self.get_logger().info(f'Updating motors and servos!')

        loop = asyncio.get_event_loop() # Runs async function in non async function!

        tasks = update_motors(velocity, steering_angle, self.motors), update_servos(steering_angle, self.motors)
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