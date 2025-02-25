import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuNoiseNode(Node):
    def __init__(self):
        super().__init__('imu_noise_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu_no_noise',
            self.imu_noise_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu', 10)

        # declare parameter for orientation_cov_factor
        self.declare_parameter('orientation_cov_factor', 0.1)
        
        # declare parameter for angular_velocity_cov_factor
        self.declare_parameter('angular_velocity_cov_factor', 0.2)
        
        # declare parameter for linear_acceleration_cov_factor
        self.declare_parameter('linear_acceleration_cov_factor', 0.2)
        

    def imu_noise_callback(self, msg):
        # Add noise to the IMU data here

        # read parameters for covariances
        orientation_cov_factor = self.get_parameter('orientation_cov_factor').value
        angular_velocity_cov_factor = self.get_parameter('angular_velocity_cov_factor').value
        linear_acceleration_cov_factor = self.get_parameter('linear_acceleration_cov_factor').value

        
        msg.orientation_covariance = np.diag(np.repeat(orientation_cov_factor, 3)).reshape(-1)
        msg.angular_velocity_covariance = np.diag(np.repeat(angular_velocity_cov_factor, 3)).reshape(-1)
        msg.linear_acceleration_covariance = np.diag(np.repeat(linear_acceleration_cov_factor, 3)).reshape(-1)

        # Publish the IMU data
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_noise_node = ImuNoiseNode()
    rclpy.spin(imu_noise_node)
    imu_noise_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
