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

#MOTOR_ROC = 33.0432965288789
MOTOR_ROC = 32.2418230613342
#MOTOR_SHIFT = -0.185685198415843
MOTOR_SHIFT = -0.155436252405753
MIN_SPEED = 0.02
class MotorDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('zs_x11_driver')
        topic_wheel = self.declare_parameter('wheel_topic', "wheel").value
        topic_motor_cmd = self.declare_parameter('motor_cmd_topic', "motor_cmd").value
        topic_wheel_vtarget = self.declare_parameter('wheel_vtarget_topic', "lwheel_vtarget").value
        pin_pwm = self.declare_parameter('pwm_pin', 13).value
        pin_reverse = self.declare_parameter('reverse_pin', 6).value
        pin_brake = self.declare_parameter('brake_pin', 26).value
        pin_speed_pulse = self.declare_parameter('speed_pulse_pin', 16).value
        inverse = self.declare_parameter('inverse_direction', False).value
# "mode" supports "CONTROLLER", "PID" (default) and "WHEEL".
# CONTROLLER takes /cmd_vel as Twist input. PID takes velocity in m/s, WHEEL takes the pwm duty cycle as input
        mode = self.declare_parameter('mode', "pid").value
        if mode == "CONTROLLER":
            from .motor_driver.motor_driver_complex import MotorDriver
            self.motor = MotorDriver()
            self.drive_twist_sub = self.create_subscription(Twist, 'cmd_vel', self.callback_drive_twist, 10)
            self.drive_twist_sub
        elif mode == "WHEEL":
            from .motor_driver.motor_driver import MotorDriver
            self.motor = MotorDriver(pwm_pin=pin_pwm, reverse_pin=pin_reverse, brake_pin=pin_brake, speed_pulse_pin = pin_speed_pulse, inverse=inverse)
            self.drive_power_sub = self.create_subscription(Float32, topic_motor_cmd, self.callback_drive_power, 10)
            self.drive_power_sub
            self.timer = self.create_timer(1.0/10, self.publish_current_speed)
        else:
            from .motor_driver.motor_driver import MotorDriver
            self.motor = MotorDriver(pwm_pin=pin_pwm, reverse_pin=pin_reverse, brake_pin=pin_brake, speed_pulse_pin = pin_speed_pulse, inverse=inverse)
            self.wheel_vtarget_sub = self.create_subscription(Float32, topic_wheel_vtarget, self.callback_wheel_vtarget, 10)
            self.wheel_vtarget_sub
            self.timer = self.create_timer(1.0/10, self.publish_tick_counter)


        self.drive_power_last_message = time.time()
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor', self.callback_stop)
        self.wheel_tick_pub = self.create_publisher(Int16, topic_wheel, 10)

        self.timer = self.create_timer(1, self.timer_callback_emergency_stop)

    def timer_callback_emergency_stop(self):
        if self.drive_power_last_message + 1 <= time.time():
            self.stop()

    def publish_tick_counter(self, event=None):
        tick_msg = Int16()
        tick_msg.data = int(self.motor.tick_counter)
        self.wheel_tick_pub.publish(tick_msg)

    def stop(self):
        self.get_logger().debug('Stopping')
        self.motor.stop()

    def callback_wheel_vtarget(self, msg):
        self.drive_power_last_message = time.time()
        input = msg.data
        if input < 0:
            self.motor.reverse()
        else:
            self.motor.reverse(False)
        if abs(input) >= MIN_SPEED:
            power = ( abs(input) + MOTOR_SHIFT ) * MOTOR_ROC
        else:
            power = 0
        self.motor.wheel(power)

    def callback_drive_power(self, msg):
        self.drive_power_last_message = time.time()
#        self.set_power = msg # shouldnt this be msg.data?
        self.motor.wheel(msg.data)

    def callback_drive_twist(self, msg):
        self.get_logger().debug(f"Received Drive Twist: {msg.linear.x} / {msg.angular.z}")
        self.drive_vector_last_message = time.time()
        self.motor.twistdrive(msg.linear.x, msg.angular.z)

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
        #safeguard used GPIO pins
        motor_driver_wrapper.motor.GPIOcleanup()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        motor_driver_wrapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
