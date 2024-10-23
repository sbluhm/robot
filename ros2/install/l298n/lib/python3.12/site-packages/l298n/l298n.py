from .motor_driver.motor_driver import MotorDriver

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
import time

class MotorDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('l298n_driver')
        self.declare_parameter('~max_speed_a', 100)
        self.declare_parameter('~scale_speed_a', 100)
        self.declare_parameter('~publish_current_speed_frequency', 5.0)
        self.declare_parameter('~publish_motor_status_frequency', 1.0)


        self.publisher_ = self.create_publisher(String, 'topic', 10)

        max_speed_a = self.get_parameter('~max_speed_a').get_parameter_value().double_value
        scale_speed_a = self.get_parameter('~scale_speed_a').get_parameter_value().integer_value
        publish_current_speed_frequency = self.get_parameter('~publish_current_speed_frequency').get_parameter_value().double_value
        publish_motor_status_frequency = self.get_parameter('~publish_motor_status_frequency').get_parameter_value().string_value

        self.motor = MotorDriver(max_speed_a=max_speed_a, scale_speed_a=scale_speed_a)
        self.speed_command_sub_a = self.create_subscription(Int32, 'speed_command_a', self.callback_speed_command_a, 10)
        self.speed_command_sub_a  # prevent unused variable warning
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor_a', self.callback_stop_a)
        self.current_speed_a_pub = self.create_publisher(Int32, "current_speed_a", 10)

        self.timer = self.create_timer(1.0/publish_current_speed_frequency, self.publish_current_speed_a)

    def publish_current_speed_a(self, event=None):
        msg = Int32()
        msg.data = int(self.motor.get_speed_a())
        self.current_speed_a_pub.publish(msg)

    def stop_a(self):
        self.get_logger().info('Stopping')
        self.motor.stop_a()

    def callback_speed_command_a(self, msg):
        self.get_logger().info('Received speed_command: "%s"' % type(msg.data))
        self.motor.set_speed_a(msg.data)

    def callback_stop_a(self, request, response):
        self.stop_a()
        response = '{"success": True, "message": "Motor has been stopped"}'
        self.get_logger().info('Incoming request: %d' % (request))
        return response


def main(args=None):
    rclpy.init(args=args)

    motor_driver_wrapper = MotorDriverROSWrapper()
    rclpy.spin(motor_driver_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
