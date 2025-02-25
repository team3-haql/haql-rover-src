from typing import Optional

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

class MockBattery(Node):
    def __init__(self):
        super().__init__('mock_battery')
        self.declare_parameter('discharge_rate', 0.1)
        self.declare_parameter('charge_rate', 0.2)
        self.declare_parameter('max_charge', 100.0)

        self._current_charge: float  = 0.0
        self._mode = 'discharging'
        self._active = None
        # self._mode_srv = self.create_service(SetBool, 'set_battery_mode', self.set_mode_callback)
        self._bat_pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None

    def set_mode_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not self._active:
            response.success = False
            response.message = 'Battery node is not active'
            return response

        if request.data:
            self._mode = 'charging'
        else:
            self._mode = 'discharging'
        response.success = True
        response.message = 'Battery mode set successfully'
        return response

    def update_charge(self):
        if self._mode == 'charging':
            charge_rate = self.get_parameter('charge_rate').value
            self._current_charge = min(self._current_charge + charge_rate, self.get_parameter('max_charge').value)
        else:
            discharge_rate = self.get_parameter('discharge_rate').value
            self._current_charge = max(self._current_charge - discharge_rate, 0.0)

        if self._bat_pub is None or not self._bat_pub.is_activated:
            self.get_logger().info(f'Publisher inactive')
        else:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = self._current_charge / self.get_parameter('max_charge').value
            if self._mode == 'charging':
                if msg.percentage > 0.95:
                    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL
                else:
                    msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING
            else:
                msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING

            msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_GOOD
            msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            msg.present = True
            self._bat_pub.publish(msg)
            self.get_logger().info(f'Battery charge: {self._current_charge}')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() is called.')

        self._mode = 'discharging'
        self._bat_pub = self.create_lifecycle_publisher(BatteryState, 'battery_charge', 10)
        self._mode_srv = self.create_service(SetBool, 'set_battery_mode', self.set_mode_callback)
        self._timer = self.create_timer(1.0, self.update_charge)
        self._current_charge = self.get_parameter('max_charge').value
        self._active = False

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate() is called.')
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate() is called.')
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._bat_pub)
        self.destroy_service(self._mode_srv)
        self._active = False

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._bat_pub)
        self.destroy_service(self._mode_srv)
        self._active = False

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    mock_battery = MockBattery()
    rclpy.spin(mock_battery)
    mock_battery.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
