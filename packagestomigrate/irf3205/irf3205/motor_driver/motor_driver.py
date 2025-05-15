import RPi.GPIO as IO

IO.setwarnings(False)
IO.setmode (IO.BCM)


# Motor PWM A
PWM1_PIN = 9
IO.setup(PWM1_PIN,IO.OUT)
motor_a = IO.PWM(PWM1_PIN,2000)
# Direction A
DIR1_PIN = 7
IO.setup(DIR1_PIN, IO.OUT)
IO.output(DIR1_PIN, False)
motor_a.start(0)

# Motor B
PWM2_PIN = 11
IO.setup(PWM2_PIN,IO.OUT)
motor_b = IO.PWM(PWM2_PIN,2000)
# Direction B
DIR2_PIN = 8
IO.setup(DIR2_PIN, IO.OUT)
IO.output(DIR2_PIN, False)


class MotorDriver:
    def __init__(self, max_speed_a=100, scale_speed_a=100, max_speed_b=100, scale_speed_b=100):
        """
        Init communication, set default settings, ...
        """
        self.max_speed_a = max_speed_a
        self.scale_speed_a = scale_speed_a
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
        if self.current_speed_a > 0:
            IO.output(DIR1_PIN, False)
        else:
            IO.output(DIR1_PIN, True)

    def set_speed_b(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if speed <= self.max_speed_b:
            self.current_speed_b = speed
        else:
            self.current_speed_b = self.max_speed_b
        motor_a.ChangeDutyCycle(self.current_speed_b)
        if self.current_speed_b > 0:
            IO.output(DIR2_PIN, False)
        else:
            IO.output(DIR2_PIN, True)

    def stop(self):
        """
        Stop all motors
        """
        self.stop_a()
        self.stop_b()
        print("All motors Stopped")

    def stop_a(self):
        """
        Set speed to 0 and thus stop motor a
        """
        motor_a.ChangeDutyCycle(0)
        IO.output(DIR1_PIN, False)
        print("Motor A Stopped")

    def stop_a(self):
        """
        Set speed to 0 and thus stop motor a
        """
        motor_b.ChangeDutyCycle(0)
        IO.output(DIR2_PIN, False)
        print("Motor B Stopped")

    def get_speed_a(self):
        """
        Return current speed
        """
        return self.current_speed_a

    def get_speed_b(self):
        """
        Return current speed
        """
        return self.current_speed_b

