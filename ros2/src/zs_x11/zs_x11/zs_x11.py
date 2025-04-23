from .motor_driver.motor_driver import MotorDriver

import rclpy
import time
from rclpy.node import Node

from custom_interfaces.msg import Vector
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Trigger

class MotorDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('zs_x11_driver')
        self.declare_parameter('~max_speed', 100)
        self.declare_parameter('~scale_speed', 100)
        self.declare_parameter('~publish_current_speed_frequency', 5.0)
        self.declare_parameter('~publish_motor_status_frequency', 1.0)

        topic_wheel = self.declare_parameter('wheel_topic', "wheel").value
        topic_motor_cmd = self.declare_parameter('motor_cmd_topic', "motor_cmd").value
        pin_pwm = self.declare_parameter('pwm_pin', 13).value
        pin_reverse = self.declare_parameter('reverse_pin', 5).value
        pin_brake = self.declare_parameter('brake_pin', 26).value


        max_speed = self.get_parameter('~max_speed').get_parameter_value().double_value
        scale_speed = self.get_parameter('~scale_speed').get_parameter_value().integer_value
        publish_current_speed_frequency = self.get_parameter('~publish_current_speed_frequency').get_parameter_value().double_value
        publish_motor_status_frequency = self.get_parameter('~publish_motor_status_frequency').get_parameter_value().string_value

        self.motor = MotorDriver(pin_pwm, pin_brake, pin_reverse, max_speed=max_speed, scale_speed=scale_speed)
        self.drive_vector_last_message = time.time()
        self.drive_vector_sub = self.create_subscription(Vector, 'drive_vector_DISABLED', self.callback_drive_vector, 10)
        self.drive_vector_sub
        self.drive_power_sub = self.create_subscription(Float32, topic_motor_cmd, self.callback_drive_power, 10)
        self.drive_power_sub
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor', self.callback_stop)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.wheel_tick_pub = self.create_publisher(Int16, topic_wheel, 10)

        self.motor_status_pub = self.create_publisher(DiagnosticStatus, "motor_status", 1)
        self.timer = self.create_timer(1.0/30, self.publish_current_speed)
        self.timer = self.create_timer(1, self.timer_callback_emergency_stop)

    def timer_callback_emergency_stop(self):
        if self.drive_vector_last_message + 1 <= time.time():
            self.stop()

    def publish_current_speed(self, event=None):
        odom_msg = Odometry()
        odom_msg.header.stamp.sec = int(time.now())
        odom_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
#        self.odom_publisher.publish(odom_msg)
        tick_msg = Int16()
        tick_msg.data = self.motor.tick_counter
        self.wheel_tick_pub.publish(tick_msg)

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

    def callback_drive_vector(self, msg):
        self.get_logger().info(f"Received Drive Vector: {msg.x}, {msg.y}")
        self.drive_vector_last_message = time.time()
        self.motor.vdrive(msg.y, msg.x)

    def callback_drive_power(self, msg):
        self.drive_vector_last_message = time.time()
        self.set_power = msg
        self.motor.wheel(msg)


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
