import rclpy
import math
import asyncio
from rclpy.node import Node
from ralphee_hardware.ralphee_motor_interface import init_motors, update_motors
from ralphee_hardware.ralphee_servo_interface import init_servos, update_servos

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int16
from ralphee_remote_control.ralphee_remote_control import Controller

WHEEL_BASE = 1.0
TRACK_WIDTH = 1.0

MAX_ANGLE = 0.75 * math.pi
MIN_ANGLE = 0.25 * math.pi

def lerp_angle(angle: float) -> float:
    """
        Translates angle to 1, -1
    """
    global MIN_ANGLE, MAX_ANGLE

    clamped_angle = max(min(angle, 1), -1)
    translated_angle = (clamped_angle + 1.0)/2.0
    return MIN_ANGLE + translated_angle*(MAX_ANGLE-MIN_ANGLE)

def get_radius(angle) -> tuple[float, float]:
    global WHEEL_BASE, TRACK_WIDTH

    angle_interpolated = lerp_angle(angle)
    denominator = math.tan(angle_interpolated)
    offset = TRACK_WIDTH/2 if angle >= 0 else -TRACK_WIDTH/2
    if denominator != 0:
        radius = (WHEEL_BASE/denominator) + offset
    else:
        radius = math.inf
    return radius

def convert_trans_rot_vel_to_radius_and_inner_angle(
    velocity: float, angular_velocity: float) -> tuple[float, float]:
    """ 
        Converts velocity and angular velocity into radius of turn and the steering angle of the turn.
        Args:
            velocity:
                x component of cmd_vel.linear
            angular_velocity:
                z comoponent of cmd_vel.angular
        Returns:
            (radius, steering_angle) 
    """
    global WHEEL_BASE, TRACK_WIDTH

    if angular_velocity == 0 or velocity == 0:
        return 0

    # Velocity is absolute since the sign of radius is used to determine which side of the rover its on.
    radius = velocity / angular_velocity
    return radius, math.atan(WHEEL_BASE / (radius - TRACK_WIDTH/2))

def inverse_lerp_angle(angle: float) -> float:
    """
        Translates angle to angle between -1 and 1
    """
    global MIN_ANGLE, MAX_ANGLE

    clamped_angle = max(min(angle, MAX_ANGLE), MIN_ANGLE)
    return (2.0*(clamped_angle - MIN_ANGLE)/(MAX_ANGLE - MIN_ANGLE)) - 1.0

class RalpheeHardwareController(Node):
    """
        Class that subscribes to 'cmd_vel' topic, retreives angle and velocity from twist object, and sends it to hardware.
    """
    def __init__(self):
        """ 
            Constructs RalpheeHardwareController class
        """
        super().__init__('ralphee_hardware_controller')
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.nav2_listener_callback, 10)
        self.controller_sub = self.create_subscription(Float64MultiArray, 'angle_vel_controller', self.controller_angle_vel_listener_callback, 10)
        self.controller_sub = self.create_subscription(Int16, 'state', self.controller_angle_vel_listener_callback, 10)
        self.state = 1

        self.get_logger().info(f'Initializing Hardware!')

        loop = asyncio.get_event_loop() # Runs async function in non async function!

        tasks = init_motors(), init_servos()
        self.motors, self.arduino = loop.run_until_complete(asyncio.gather(*tasks))

        loop.close()

        self.get_logger().info(f'Initialized Hardware!')

    def nav2_listener_callback(self, msg: Twist):
        """
            Receives velocity and angle and sends it to hardware!
            Args:
                msg:
                    Twist object that comes with angle and velocity.
        """
        if self.state != 2:
            return
        velocity: float = msg.linear.x
        angular_velocity: float = msg.angular.z
        radius, inner_angle = convert_trans_rot_vel_to_radius_and_inner_angle(velocity, angular_velocity)

        inner_angle_n1_to_1 = inverse_lerp_angle(inner_angle)

        self.get_logger().info(f'Updating motors and servos!')

        loop = asyncio.get_event_loop() # Runs async function in non async function!

        tasks = (
            update_motors(velocity, inner_angle_n1_to_1, radius, self.motors), 
            update_servos(inner_angle_n1_to_1, self.motors)
        )
        self.motors, self.arduino = loop.run_until_complete(asyncio.gather(*tasks))

        loop.close()

        self.get_logger().info(f'Updated motors and servos!')

    def controller_state_listener_callback(self, msg: Int16):
        if msg.data == 1:
            self.state = 1
        elif msg.data == 2:
            self.state = 2

    def controller_angle_vel_listener_callback(self, msg: Float64MultiArray):
        if self.state != 1:
            return
        vel = msg[0]
        angle = msg[1]

        radius = get_radius(radius)

        loop = asyncio.get_event_loop() # Runs async function in non async function!

        tasks = (
            update_motors(vel, angle, radius, self.motors), 
            update_servos(vel, self.motors)
        )
        self.motors, self.arduino = loop.run_until_complete(asyncio.gather(*tasks))

        loop.close()

        

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