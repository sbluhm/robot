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
        super().__init__('integrated_driver')
        topic_lwheel = self.declare_parameter('lwheel_topic', "lwheel").value
        topic_lmotor_cmd = self.declare_parameter('lmotor_cmd_topic', "lmotor_cmd").value
        topic_lwheel_vtarget = self.declare_parameter('lwheel_vtarget_topic', "lwheel_vtarget").value
        topic_rwheel = self.declare_parameter('rwheel_topic', "rwheel").value
        topic_rmotor_cmd = self.declare_parameter('rmotor_cmd_topic', "rmotor_cmd").value
        topic_rwheel_vtarget = self.declare_parameter('rwheel_vtarget_topic', "rwheel_vtarget").value
        pin_lpwm = self.declare_parameter('lpwm_pin', 13).value
        pin_lreverse = self.declare_parameter('lreverse_pin', 6).value
        pin_lbrake = self.declare_parameter('lbrake_pin', 26).value
        pin_lspeed_pulse = self.declare_parameter('lspeed_pulse_pin', 16).value
        pin_rpwm = self.declare_parameter('rpwm_pin', 12).value
        pin_rreverse = self.declare_parameter('rreverse_pin', 5).value
        pin_rbrake = self.declare_parameter('rbrake_pin', 25).value
        pin_rspeed_pulse = self.declare_parameter('rspeed_pulse_pin', 19).value
        linverse = self.declare_parameter('linverse_direction', False).value
        rinverse = self.declare_parameter('rinverse_direction', True).value

# "mode" supports "CONTROLLER", "PID" (default) and "WHEEL".
# CONTROLLER takes /cmd_vel as Twist input. PID takes velocity in m/s, WHEEL takes the pwm duty cycle as input
        mode = self.declare_parameter('mode', "pid").value
        from .motor_driver.motor_driver import MotorDriver

        self.lmotor = MotorDriver(pwm_pin=lpin_pwm, reverse_pin=pin_lreverse, brake_pin=pin_lbrake, speed_pulse_pin = pin_lspeed_pulse, inverse=linverse)
        self.lwheel_vtarget_sub = self.create_subscription(Float32, topic_lwheel_vtarget, self.callback_lwheel_vtarget, 10)
        self.lwheel_vtarget_sub
        self.lwheel_tick_pub = self.create_publisher(Int16, topic_lwheel, 10)

        self.rmotor = MotorDriver(pwm_pin=pin_rpwm, reverse_pin=pin_rreverse, brake_pin=pin_rbrake, speed_pulse_pin = pin_rspeed_pulse, inverse=rinverse)
        self.rwheel_vtarget_sub = self.create_subscription(Float32, topic_rwheel_vtarget, self.callback_rwheel_vtarget, 10)
        self.rwheel_vtarget_sub
        self.rwheel_tick_pub = self.create_publisher(Int16, topic_rwheel, 10)

        self.timer = self.create_timer(1.0/10, self.publish_tick_counter)
        self.drive_power_last_message = time.time()
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor', self.callback_stop)
        self.timer = self.create_timer(1, self.timer_callback_emergency_stop)

    def timer_callback_emergency_stop(self):
        if self.drive_power_last_message + 1 <= time.time():
            self.stop()

    def publish_tick_counter(self, event=None):
        ltick_msg = Int16()
        ltick_msg.data = int(self.lmotor.tick_counter)
        self.lwheel_tick_pub.publish(ltick_msg)
        rtick_msg = Int16()
        rtick_msg.data = int(self.rmotor.tick_counter)
        self.rwheel_tick_pub.publish(rtick_msg)



    def stop(self):
        self.get_logger().debug('Stopping')
        self.lmotor.stop()
        self.rmotor.stop()

    def callback_lwheel_vtarget(self, msg):
        self.drive_power_last_message = time.time()
        input = msg.data
        if input < 0:
            self.lmotor.reverse()
        else:
            self.lmotor.reverse(False)
        if abs(input) >= MIN_SPEED:
            power = ( abs(input) + MOTOR_SHIFT ) * MOTOR_ROC
        else:
            power = 0
        self.lmotor.wheel(power)
    def callback_rwheel_vtarget(self, msg):
        self.drive_power_last_message = time.time()
        input = msg.data
        if input < 0:
            self.rmotor.reverse()
        else:
            self.rmotor.reverse(False)
        if abs(input) >= MIN_SPEED:
            power = ( abs(input) + MOTOR_SHIFT ) * MOTOR_ROC
        else:
            power = 0
        self.rmotor.wheel(power)

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
