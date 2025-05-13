import rclpy
import time

from .motor_driver.motor_driver import MotorDriver
from contextlib import suppress
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

class MotorDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('l298n_driver')
        self.declare_parameter('~max_speed_a', 50)
        self.declare_parameter('~step_speed_a', 1)
        self.declare_parameter('~publish_current_speed_frequency', 5.0)
        self.declare_parameter('~publish_motor_status_frequency', 1.0)

        self.current_speed_a = 0

        self.publisher_ = self.create_publisher(String, 'topic', 10)

        max_speed_a = self.get_parameter('~max_speed_a').get_parameter_value().integer_value
        self.step_speed_a = self.get_parameter('~step_speed_a').get_parameter_value().integer_value
        publish_current_speed_frequency = self.get_parameter('~publish_current_speed_frequency').get_parameter_value().double_value
        publish_motor_status_frequency = self.get_parameter('~publish_motor_status_frequency').get_parameter_value().string_value

        self.motor = MotorDriver(max_speed_a=max_speed_a)
        self.speed_command_sub_a = self.create_subscription(Int32, 'speed_command_a', self.callback_speed_command_a, 10)
        self.speed_command_sub_a  # prevent unused variable warning
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor_a', self.callback_stop_a)
        self.current_speed_a_pub = self.create_publisher(Int32, "current_speed_a", 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.callback_joy, 10)
        self.joy_sub  # prevent unused variable warning


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

    def callback_joy(self, joy_msg):
#        self.current_speed_a
        if( joy_msg.buttons[11] ):
            self.current_speed_a += self.step_speed_a
        if( joy_msg.buttons[12] ):
            self.current_speed_a -= self.step_speed_a
        if( self.current_speed_a < 0):
            self.current_speed_a = 0
        if( self.current_speed_a > 100):
            self.current_speed_a = 100

        self.motor.set_speed_a(self.current_speed_a)

        print(self.current_speed_a)
#        self.drive_vector_pub.publish(pub_msg)
#        self.get_logger().info('Submitted from Joy to Drive')


def main(args=None):
    rclpy.init(args=args)
    motor_driver_wrapper = MotorDriverROSWrapper()

    try:
        while rclpy.ok():
            rclpy.spin_once(motor_driver_wrapper)
            motor_driver_wrapper.create_rate(2).sleep()
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
