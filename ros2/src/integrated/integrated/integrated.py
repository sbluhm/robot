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
from geometry_msgs.msg import Twist


#MOTOR_ROC = 33.0432965288789
MOTOR_ROC = 32.2418230613342
#MOTOR_SHIFT = -0.185685198415843
MOTOR_SHIFT = -0.155436252405753
MIN_SPEED = 0.02
class MotorDriverWrapper(Node):

    def __init__(self):
        super().__init__('integrated_driver')
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

        from .motor_driver.motor_driver import MotorDriver

        self.lmotor = MotorDriver(pwm_pin=pin_lpwm, reverse_pin=pin_lreverse, brake_pin=pin_lbrake, speed_pulse_pin = pin_lspeed_pulse, inverse=linverse)
        self.rmotor = MotorDriver(pwm_pin=pin_rpwm, reverse_pin=pin_rreverse, brake_pin=pin_rbrake, speed_pulse_pin = pin_rspeed_pulse, inverse=rinverse)

        self.drive_power_last_message = time.time()
        self.stop_motor_srv = self.create_service(Trigger, 'stop_motor', self.callback_stop)
        self.timer = self.create_timer(1, self.timer_callback_emergency_stop)

    def timer_callback_emergency_stop(self):
        if self.drive_power_last_message + 1 <= time.time():
            self.stop()


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



class TwistToMotors(Node):
    """ 
    twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

    """

    def __init__(self):
        super(TwistToMotors, self).__init__("twist_to_motors")
        self.nodename = "twist_to_motors"
    
        ### get parameters ####
        topic_twist = self.declare_parameter('twist_topic', "cmd_vel_smoothed").value
        topic_lwheel = self.declare_parameter('lwheel_topic', "lwheel").value
        topic_rwheel = self.declare_parameter('rwheel_topic', "rwheel").value

        self.get_logger().info("%s started" % self.nodename)

        self.w = self.declare_parameter("base_width", 0.2).value
        self.dx = 0
        self.dr = 0
        self.ticks_since_target = 0
    
        self.create_subscription(Twist, topic_twist, self.twist_callback, 10)
        self.motor = MotorDriverWrapper()

        self.lwheel_tick_pub = self.create_publisher(Int16, topic_lwheel, 10)
        self.rwheel_tick_pub = self.create_publisher(Int16, topic_rwheel, 10)

#        self.rate_hz = self.declare_parameter("rate_hz", 50).value
        self.rate_hz = self.declare_parameter("rate_hz", 5).value       
        self.create_timer(1.0/self.rate_hz, self.calculate_left_and_right_target)
        self.timer = self.create_timer(1.0/10, self.publish_tick_counter)

    def calculate_left_and_right_target(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        right = Float32()
        left = Float32()
        
        right.data = 1.0 * self.dx + self.dr * self.w / 2.0
        left.data = 1.0 * self.dx - self.dr * self.w / 2.0

        self.motor.callback_lwheel_vtarget(left)
        self.motor.callback_rwheel_vtarget(right)

        self.ticks_since_target += 1


    def twist_callback(self, msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z

    def publish_tick_counter(self, event=None):
        ltick_msg = Int16()
        ltick_msg.data = int(self.motor.lmotor.tick_counter)
        self.lwheel_tick_pub.publish(ltick_msg)
        rtick_msg = Int16()
        rtick_msg.data = int(self.motor.rmotor.tick_counter)
        self.rwheel_tick_pub.publish(rtick_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        twist_to_motors = TwistToMotors()
        rclpy.spin(twist_to_motors)

    except KeyboardInterrupt:
        pass

    with suppress(Exception):
        #safeguard used GPIO pins
        motor_driver_wrapper.motor.GPIOcleanup()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        twist_to_motors.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
