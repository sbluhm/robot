from .motor_driver.motor_driver import MotorDriver

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import time
from custom_interfaces.msg import Vector
from diagnostic_msgs.msg import DiagnosticStatus

class MotorDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('zs_x11_driver')
        self.declare_parameter('~max_speed', 100)
        self.declare_parameter('~scale_speed', 100)
        self.declare_parameter('~publish_current_speed_frequency', 5.0)
        self.declare_parameter('~publish_motor_status_frequency', 1.0)


        self.publisher_ = self.create_publisher(String, 'topic', 10)

        max_speed = self.get_parameter('~max_speed').get_parameter_value().double_value
        scale_speed = self.get_parameter('~scale_speed').get_parameter_value().integer_value
        publish_current_speed_frequency = self.get_parameter('~publish_current_speed_frequency').get_parameter_value().double_value
        publish_motor_status_frequency = self.get_parameter('~publish_motor_status_frequency').get_parameter_value().string_value

        self.motor = MotorDriver(max_speed=max_speed, scale_speed=scale_speed)
        self.drive_vector_last_message = time.time()
        self.drive_vector_sub = self.create_subscription(Vector, 'drive_vector_DISABLED', self.callback_drive_vector, 10)
        self.drive_vector_sub
        self.drive_twist_sub = self.create_subscription(Twist, 'cmd_vel', self.callback_drive_twist, 10)
        self.drive_twist_sub
        self.speed_command_sub = self.create_subscription(Int32, 'speed_command', self.callback_speed_command, 10)
        self.speed_command_sub  # prevent unused variable warning
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor', self.callback_stop)
        self.current_speed_pub = self.create_publisher(Int32, "current_speed", 10)

        self.motor_status_pub = self.create_publisher(DiagnosticStatus, "motor_status", 1)
        self.timer = self.create_timer(1.0/publish_current_speed_frequency, self.publish_current_speed)
        self.timer = self.create_timer(1, self.timer_callback_emergency_stop)

    def timer_callback_emergency_stop(self):
        if self.drive_vector_last_message + 1 <= time.time():
            self.stop()

    def publish_current_speed(self, event=None):
        msg = Int32()
        msg.data = int(self.motor.get_speed())
        self.current_speed_pub.publish(msg)

    def publish_motor_status(self, event=None):
        status = self.motor.get_status()
        data_list = []
        for key in status:
            data_list.append(KeyValue(key, str(status[key])))
        msg = DiagnosticStatus()
        msg.values = data_list
        self.motor_status_pub.publish(msg)

    def stop(self):
        self.get_logger().info('Stopping')
        self.motor.stop()

    def callback_speed_command(self, msg):
        self.get_logger().info('Received speed_command: "%s"' % type(msg.data))
        self.motor.set_speed(msg.data)

    def callback_drive_vector(self, msg):
        self.get_logger().info(f"Received Drive Vector: {msg.x}, {msg.y}")
        self.drive_vector_last_message = time.time()
        self.motor.vdrive(msg.y, msg.x)

    def callback_drive_twist(self, msg):
        self.get_logger().info(f"Received Drive Twist: {msg.linear.x} / {msg.angular.z}")
        self.drive_vector_last_message = time.time()
        self.motor.twistdrive(msg.linear.x, msg.angular.z)


    def callback_stop(self, request, response):
        self.stop()
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
