import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import tf2_ros
import yaml
import os

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('waypoints_file', 'waypoints.yml')
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.waypoints = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(Trigger, 'record_waypoint', self.record_waypoint_callback)

    def record_waypoint_callback(self, request, response):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            waypoint = {'x': trans.transform.translation.x, 'y': trans.transform.translation.y, 'z': trans.transform.translation.z}
            self.waypoints.append(waypoint)
            self.write_waypoints_to_file()
            response.success = True
            response.message = "Waypoint recorded"
        except Exception as e:
            self.get_logger().error('Failed to record waypoint: "%s"' % str(e))
            response.success = False
            response.message = str(e)
        return response

    def write_waypoints_to_file(self):
        with open(self.waypoints_file, 'w') as file:
            yaml.dump(self.waypoints, file)

def main(args=None):
    rclpy.init(args=args)
    waypoint_recorder = WaypointRecorder()
    rclpy.spin(waypoint_recorder)
    waypoint_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
