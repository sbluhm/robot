#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPoseListener(Node):
    def __init__(self):
        super().__init__('initial_pose_listener')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Convert quaternion to yaw
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        command = f"ros2 run tf2_ros static_transform_publisher {x:.3f} {y:.3f} 0 0 0 {yaw:.3f} map odom"
        self.get_logger().info(f"Static Transform Publisher Command:\n{command}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

