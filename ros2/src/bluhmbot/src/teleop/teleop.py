#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import AssistedTeleop
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped

class AssistedTeleopKeepAlive(Node):
    def __init__(self):
        super().__init__('assisted_teleop_keepalive')
        self._client = ActionClient(self, AssistedTeleop, '/assisted_teleop')

        self._goal_timer = None
        self._watchdog_timer = None
        self._active = False

        self._subscriber = self.create_subscription(
            TwistStamped,
            '/cmd_vel_teleop',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        if not self._active:
            self.get_logger().info('Received /cmd_vel_teleop. Starting goal timer.')
            self._active = True
            self.start_goal_timer()

        self.reset_watchdog_timer()

    def start_goal_timer(self):
        if self._goal_timer is None:
            self._goal_timer = self.create_timer(9.0, self.send_goal)
            self.get_logger().info('Goal timer started.')

    def stop_goal_timer(self):
        if self._goal_timer:
            self._goal_timer.cancel()
            self.destroy_timer(self._goal_timer)
            self._goal_timer = None
            self.get_logger().info('Goal timer stopped.')

    def reset_watchdog_timer(self):
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self.destroy_timer(self._watchdog_timer)

        self._watchdog_timer = self.create_timer(9.0, self.watchdog_expired)
        self.get_logger().debug('Watchdog timer reset.')

    def watchdog_expired(self):
        self.get_logger().info('No /cmd_vel_teleop received in 9 seconds. Stopping goal timer.')
        self._active = False
        self.stop_goal_timer()

        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self.destroy_timer(self._watchdog_timer)
            self._watchdog_timer = None

    def send_goal(self):
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('AssistedTeleop action server not available.')
            return

        goal_msg = AssistedTeleop.Goal()
        goal_msg.time_allowance = Duration(sec=10)

        self.get_logger().info('Sending AssistedTeleop goal...')
        self._client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AssistedTeleopKeepAlive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

