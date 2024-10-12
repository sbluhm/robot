import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from custom_interfaces.msg import Vector

class MotorControllerROSWrapper(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.callback_joy, 10)
        self.joy_sub  # prevent unused variable warning

        self.drive_vector_pub = self.create_publisher(Vector, "drive_vector", 1)

    def callback_joy(self, joy_msg):
        pub_msg = Vector()
        pub_msg.x = -1*joy_msg.axes[2]
        pub_msg.y = joy_msg.axes[1]
        self.drive_vector_pub.publish(pub_msg)
#        self.get_logger().info('Submitted from Joy to Drive')

def main(args=None):
    rclpy.init(args=args)

    motor_controller_wrapper = MotorControllerROSWrapper()
    rclpy.spin(motor_controller_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver_wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
