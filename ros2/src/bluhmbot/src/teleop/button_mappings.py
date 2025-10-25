#!/usr/bin/env python3
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import AssistedTeleop
from rclpy.action import ActionClient
import yaml
import os

class JoyButtonMapper(Node):
    def __init__(self):
        super().__init__('joy_button_mapper')

        # Load YAML config
        config_path = os.path.join(
            get_package_share_directory('bluhmbot'),
            'config',
            'button_mappings.yaml'
        )
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)['button_mappings']

        self.last_button_states = {}

        # Setup publishers and action clients
        self.button_publishers = {}
        self.action_clients = {}

        for name, mapping in self.config.items():
            if 'topic' in mapping:
                msg_type = self.get_msg_type(mapping['message_type'])
                self.button_publishers[name] = self.create_publisher(msg_type, mapping['topic'], 10)
            elif 'action_server' in mapping:
                self.action_clients[name] = ActionClient(self, AssistedTeleop, mapping['action_server'])

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def get_msg_type(self, type_str):
        if type_str == 'std_msgs/msg/Bool':
            return Bool
        elif type_str == 'std_msgs/msg/Float64MultiArray':
            return Float64MultiArray 
        elif type_str == 'geometry_msgs/msg/Twist':
            return Twist
        else:
            raise ValueError(f"Unsupported message type: {type_str}")

    def joy_callback(self, msg):
        for name, mapping in self.config.items():
            idx = mapping['button_index']
            pressed = msg.buttons[idx] == 1
            last_state = self.last_button_states.get(name, False)

            if pressed and not last_state:
                self.get_logger().info(f"Button {idx} pressed for action '{name}'")

                if 'topic' in mapping:
                    msg_obj = self.create_message(mapping['message_type'], mapping['message_data'])
                    self.button_publishers[name].publish(msg_obj)

                elif 'action_server' in mapping:
                    goal_msg = AssistedTeleop.Goal()
                    goal_msg.time_allowance.sec = mapping['goal']['time_allowance']['sec']
                    goal_msg.time_allowance.nanosec = mapping['goal']['time_allowance']['nanosec']
                    self.action_clients[name].send_goal_async(goal_msg)

            self.last_button_states[name] = pressed

    def create_message(self, type_str, data):
        if type_str == 'std_msgs/msg/Bool':
            msg = Bool()
            msg.data = data
            return msg
        elif type_str == 'std_msgs/msg/Float64MultiArray':
            msg = Float64MultiArray()
            msg.data = [data]
            return msg
        elif type_str == 'geometry_msgs/msg/Twist':
            msg = Twist()
            msg.linear.x = data['linear']['x']
            msg.linear.y = data['linear']['y']
            msg.linear.z = data['linear']['z']
            msg.angular.x = data['angular']['x']
            msg.angular.y = data['angular']['y']
            msg.angular.z = data['angular']['z']
            return msg

def main(args=None):
    rclpy.init(args=args)
    node = JoyButtonMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
