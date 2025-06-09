#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient

class CoverageStartNavigator(Node):
    def __init__(self):
        super().__init__('coverage_start_navigator')
        self.path_subscriber = self.create_subscription(
            Path,
            '/coverage_server/coverage_plan',  # Replace with your actual topic
            self.path_callback,
            10
        )
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.coverage_path = None

    def path_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn('Received empty coverage path.')
            return

        self.coverage_path = msg
        self.get_logger().info('Received coverage path. Navigating to start...')
        self.navigate_to_start()

    def navigate_to_start(self):
        if not self.coverage_path:
            self.get_logger().warn('No path available.')
            return

        start_pose = self.coverage_path.poses[0]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = start_pose

        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Sending goal to NavigateToPose...')
        self.nav_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoverageStartNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

