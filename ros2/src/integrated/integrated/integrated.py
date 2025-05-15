import rclpy
import time
from .motor_driver.motor_driver import MotorDriver
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

class TwistToMotors(Node):
    """ 
    twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

    """

    def __init__(self):
        super(TwistToMotors, self).__init__("integrated")
        self.nodename = "integrated"
    
        ### get parameters ####
        topic_twist = self.declare_parameter('twist_topic', "cmd_vel_smoothed").value
        topic_lwheel = self.declare_parameter('lwheel_topic', "lwheel").value
        topic_rwheel = self.declare_parameter('rwheel_topic', "rwheel").value
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


        self.rate_odom_hz = self.declare_parameter("rate_odom_hz", 10.0).value # the rate at which to publish the transform
        self.create_timer(1.0/self.rate_odom_hz, self.update)

        self.ticks_meter = float(
            self.declare_parameter('ticks_meter', 90).value)  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(self.declare_parameter('base_width', 0.46).value)  # The wheel base width in meters

        self.base_frame_id = self.declare_parameter('base_frame_id',
                                                    'base_footprint').value  # the name of the base frame of the robot
        self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                    'odom').value  # the name of the odometry reference frame

        self.encoder_min = self.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.declare_parameter('encoder_max', 32768).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value


        self.lmotor = MotorDriver(pwm_pin=pin_lpwm, reverse_pin=pin_lreverse, brake_pin=pin_lbrake, speed_pulse_pin = pin_lspeed_pulse, inverse=linverse)
        self.rmotor = MotorDriver(pwm_pin=pin_rpwm, reverse_pin=pin_rreverse, brake_pin=pin_rbrake, speed_pulse_pin = pin_rspeed_pulse, inverse=rinverse)

        self.get_logger().info("%s started" % self.nodename)

        # internal data
        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.left = 0.0  # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()

        self.w = self.declare_parameter("base_width", 0.2).value
        self.dx = 0
        self.dr = 0
        self.ticks_since_target = 0
    
        self.create_subscription(Twist, topic_twist, self.twist_callback, 10)
        self.motor = MotorDriverWrapper()

#        self.rate_hz = self.declare_parameter("rate_hz", 50).value
        self.rate_hz = self.declare_parameter("rate_hz", 5).value
        self.create_timer(1.0/self.rate_hz, self.calculate_left_and_right_target)

    def calculate_left_and_right_target(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        right = Float32()
        left = Float32()

        if self.ticks_since_target > self.rate_hz:
            right.data = 0.0
            left.data = 0.0
        else:
            right.data = 1.0 * self.dx + self.dr * self.w / 2.0
            left.data = 1.0 * self.dx - self.dr * self.w / 2.0

        self.motor.callback_lwheel_vtarget(left)
        self.motor.callback_rwheel_vtarget(right)

        self.ticks_since_target += 1

        update_tick_counter(self.lmotor.tick_counter, self.prev_lencoder, self.lmult, self.left)
        update_tick_counter(self.rmotor.tick_counter, self.prev_rencoder, self.rmult, self.right)

    def callback_lwheel_vtarget(self, msg):
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

    def twist_callback(self, msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z

    def update_tick_counter(self, tick_counter, prev_encoder, mult, wheel):
        env = int(tick_counter)
        if enc < self.encoder_low_wrap and prev_encoder > self.encoder_high_wrap:
            mult = mult + 1

        if enc > self.encoder_high_wrap and prev_encoder < self.encoder_low_wrap:
            mult = mult - 1

        wheel = 1.0 * (enc + mult * (self.encoder_max - self.encoder_min))
        prev_encoder = enc



   def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right

        # distance traveled is the average of the two wheels
        d = (d_left + d_right) / 2
        # this approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            # calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w

        self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)


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
