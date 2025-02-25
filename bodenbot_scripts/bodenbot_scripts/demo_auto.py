import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from boden_interfaces.action import Dock
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action import ActionClient
from nav_msgs.msg import Path
import rclpy.qos


class YamlWaypointParser:
    """
    Parse a set of waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

        routes = []
        for route in self.wps_dict['routes']:
            routes.append(route)

        route_waypoints = {}

        for route in routes:
            waypoints = []
            for wp in self.wps_dict[route]:
                x, y, z = wp['x'], wp['y'], wp['z']
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                waypoints.append(pose)

            route_waypoints[route] = waypoints

        self.routes = routes
        self.route_waypoints = route_waypoints
        self.index = 0

    def get_wps(self):
        route = self.routes[self.index]
        self.index = (self.index + 1) % len(self.routes)
        print(f'loaded route: {route}')

        waypoints = self.route_waypoints[route]
        return route, waypoints


class DemoAuto:
    """
    Class to use run demo autonomous code
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator('basic_navigator')
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.action_client = ActionClient(self.navigator, Dock, 'dock')
        self.waypoint_publisher = self.navigator.create_publisher(Path, '/waypoint_path', 10)
        self.cmd_puplisher = self.navigator.create_publisher(Twist, '/cmd_vel', 10)

    def dock(self):
        # TODO: make sure alignment is correct
        self.action_client.wait_for_server()
        self._docking_response_future = self.action_client.send_goal_async(Dock.Goal(), feedback_callback=self.docking_feedback_callback)
        self._docking_response_future.add_done_callback(self.docking_response_callback)
        self._docking_complete = False

        rclpy.spin_until_future_complete(self.navigator, self._docking_response_future)
        while not self._docking_complete:
            time.sleep(0.1)
            rclpy.spin_once(self.navigator)

    def docking_response_callback(self, future: rclpy.Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.navigator.get_logger().info('Docking Goal Rejected')
            return
        
        self.navigator.get_logger().info('Docking Goal Started')
        self._docking_result_future = goal_handle.get_result_async()
        self._docking_result_future.add_done_callback(self.docking_result_callback)
        rclpy.spin_until_future_complete(self.navigator, self._docking_result_future)

    def docking_feedback_callback(self, feedback_msg):
        # self.navigator.get_logger().info('info')
        pass

    def docking_result_callback(self, future: rclpy.Future):
        self._docking_complete = True
        self.navigator.get_logger().info('dock complete')

    def undock(self):
        # run navigation task
        distance = 3.0
        speed = 0.15
        wait_time = distance / speed
        for _ in range(int(wait_time * 10)):
            msg = Twist()
            msg.linear.x = -speed
            self.cmd_puplisher.publish(msg)
            time.sleep(0.1)
        self.navigator.get_logger().info('undock complete')

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        route, wps = self.wp_parser.get_wps()
        self.navigator.get_logger().info(f'loading route: {route}')
        msg = Path()
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.poses = wps
        self.waypoint_publisher.publish(msg)
        self.navigator.get_logger().info('published!')
        self.navigator.followWaypoints(wps)
        self.navigator.get_logger().info('starting!')
        # run navigation task
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        self.navigator.get_logger().info('wps path complete')

def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(
        get_package_share_directory('bodenbot_scripts'), 'config', 'waypoints_mines.yml'
    )
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = DemoAuto(yaml_file_path)

    gps_wpf.navigator.get_logger().info('STARTING AUTONOMOUS DEMO')

    for _ in range(3):
        # gps_wpf.undock()
        # time.sleep(1)
        gps_wpf.start_wpf()
        time.sleep(1)
        # gps_wpf.dock()
        # time.sleep(10)

    gps_wpf.navigator.get_logger().info('EXIT')


if __name__ == '__main__':
    main()
