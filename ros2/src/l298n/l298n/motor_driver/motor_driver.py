import RPi.GPIO as IO

IO.setwarnings(False)
IO.setmode (IO.BCM)

IO.setup(8,IO.OUT)

motor_a = IO.PWM(8,8000)
#IN01
IO.setup(7, IO.OUT)
IO.output(7, False)
#IN02
IO.setup(1, IO.OUT)
IO.output(1, False)
motor_a.start(0)


class MotorDriver:
    def __init__(self, max_speed_a=50, max_speed_b=100, scale_speed_b=100):
        """
        Init communication, set default settings, ...
        """
        self.max_speed_a = max_speed_a
        self.current_speed_a = 0

    def set_speed_a(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if speed <= self.max_speed_a:
            self.current_speed_a = speed
        else:
            self.current_speed_a = self.max_speed_a
        motor_a.ChangeDutyCycle(self.current_speed_a)
        print(f"Current brush speed = {self.current_speed_a}")
        if self.current_speed_a < 0:
            IO.output(1, True)
            IO.output(7, False)
        else:
            IO.output(1, False)
            IO.output(7, True)

    def stop(self):
        """
        Stop all motors
        """
        self.stop_a()
        print("All motors Stopped")

    def stop_a(self):
        """
        Set speed to 0 and thus stop motor a
        """
        motor_a.ChangeDutyCycle(0)
        IO.output(1, False)
        IO.output(7, False)
        print("Motor A Stopped")

    def get_speed_a(self):
        """
        Return current speed
        """
        return self.current_speed_a
