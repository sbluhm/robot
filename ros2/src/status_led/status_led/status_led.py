import rclpy
import RPi.GPIO as IO
import time

from contextlib import suppress
from rclpy.node import Node
from sensor_msgs.msg import Joy

IO.setwarnings(False)
IO.setmode (IO.BCM)

#Ready LED
LED=27
IO.setup(LED, IO.OUT)
IO.output(LED, False)


class StatusLedROSWrapper(Node):

    def __init__(self):
        super().__init__('status_led')
        self.joy_last_message = 0

        self.led_flash()

        self.joy_sub = self.create_subscription(Joy, 'joy', self.callback_joy, 10)
        self.joy_sub  # prevent unused variable warning

        self.timer = self.create_timer(1, self.timer_check_system_status)

    def callback_joy(self, joy_msg):
        self.joy_last_message = time.time()
        self.led_on()

    def timer_check_system_status(self):
        if self.joy_last_message + 1 <= time.time():
            self.led_flash()

    def led_flash(self):
        IO.output(LED, True)
        time.sleep(0.5)
        IO.output(LED, False)
        time.sleep(0.5)

    def led_on(self):
        IO.output(LED, True)

    def GPIOcleanup(self):
        IO.cleanup()


def main(args=None):
    rclpy.init(args=args)

    try:
        status_led_wrapper = StatusLedROSWrapper()
        rclpy.spin(status_led_wrapper)
    except KeyboardInterrupt:
        pass

    with suppress(Exception):
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        status_led_wrapper.GPIOcleanup()
        status_led_wrapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

