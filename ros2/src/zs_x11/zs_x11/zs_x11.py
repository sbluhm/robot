from .motor_driver.motor_driver import MotorDriver

import rclpy
import time
from contextlib import suppress
from custom_interfaces.msg import Vector
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Trigger

class MotorDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('zs_x11_driver')
        self.declare_parameter('~publish_current_speed_frequency', 5.0)
        self.declare_parameter('~publish_motor_status_frequency', 1.0)

        topic_wheel = self.declare_parameter('wheel_topic', "wheel").value
        topic_motor_cmd = self.declare_parameter('motor_cmd_topic', "motor_cmd").value
        pin_pwm = self.declare_parameter('pwm_pin', 13).value
        pin_reverse = self.declare_parameter('reverse_pin', 6).value
        pin_brake = self.declare_parameter('brake_pin', 26).value

        publish_current_speed_frequency = self.get_parameter('~publish_current_speed_frequency').get_parameter_value().double_value
        publish_motor_status_frequency = self.get_parameter('~publish_motor_status_frequency').get_parameter_value().string_value

        self.motor = MotorDriver(pwm_pin=pin_pwm, reverse_pin=pin_reverse, brake_pin=pin_brake)
        self.drive_power_last_message = time.time()
        self.drive_power_sub = self.create_subscription(Float32, topic_motor_cmd, self.callback_drive_power, 10)
        self.drive_power_sub
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor', self.callback_stop)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.wheel_tick_pub = self.create_publisher(Int16, topic_wheel, 10)

        self.motor_status_pub = self.create_publisher(DiagnosticStatus, "motor_status", 1)
        self.timer = self.create_timer(1.0/1, self.publish_current_speed)
        self.timer = self.create_timer(1, self.timer_callback_emergency_stop)

    def timer_callback_emergency_stop(self):
        if self.drive_power_last_message + 1 <= time.time():
            self.stop()

    def publish_current_speed(self, event=None):
        tick_msg = Int16()
        tick_msg.data = int(self.motor.tick_counter)
        self.wheel_tick_pub.publish(tick_msg)

    def stop(self):
        self.get_logger().debug('Stopping')
        self.motor.stop()

    def callback_drive_power(self, msg):
        self.drive_power_last_message = time.time()
        self.set_power = msg
        self.motor.wheel(msg.data)

    def callback_stop(self, request, response):
        self.stop()
        response = '{"success": True, "message": "Motor has been stopped"}'
        self.get_logger().info('Incoming request: %d' % (request))
        return response

def main(args=None):
    try:
        rclpy.init(args=args)

        motor_driver_wrapper = MotorDriverROSWrapper()
        rclpy.spin(motor_driver_wrapper)

    except KeyboardInterrupt:
        pass

    with suppress(Exception):
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        motor_driver_wrapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
