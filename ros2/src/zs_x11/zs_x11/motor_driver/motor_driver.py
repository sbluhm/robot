import RPi.GPIO as IO
import time

MAX_POWER_VALUE_SCALE = 100.0/255.0

IO.setwarnings(False)
IO.setmode (IO.BCM)

class MotorDriver:
    def __init__(self, pwm_pin=13, reverse_pin=6, brake_pin=26, speed_pulse_pin=16, inverse=False):
        """
        Init communication, set default settings, ...
        """
        self.tick_counter = 0
        self.last_tick_time = time.time()
        # Used to add/subtract ticks positive or negative
        self.__direction = 1
        self.inverse = inverse
        if inverse:
            self.inverse_multiplier = -1

    # left defaults
        self.pwm_pin = pwm_pin
        self.reverse_pin = reverse_pin
        self.brake_pin = brake_pin

        IO.setup(self.pwm_pin, IO.OUT)
        IO.setup(self.reverse_pin, IO.OUT)
        IO.setup(self.brake_pin, IO.OUT)
        IO.setup(speed_pulse_pin, IO.IN)
        IO.add_event_detect(speed_pulse_pin, IO.RISING, 
            callback=self.speed_pulse_callback, bouncetime=10)
        self.motor = IO.PWM(self.pwm_pin,10000)
        self.motor.start(0)

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        self.motor.ChangeDutyCycle(0)
        IO.output(self.brake_pin, True)

    def speed_pulse_callback(self, interrupt_pin):
        self.tick_counter = self.tick_counter + self.__direction

    def wheel(self, power):
        # Release brakes
        IO.output(self.brake_pin, False)

        # Set direction and scale to 255
        power = power * MAX_POWER_VALUE_SCALE
        if power < 0:
            self.__direction = -1
            IO.output(self.reverse_pin, not self.inverse)
        else:
            self.__direction = 1
            IO.output(self.reverse_pin, self.inverse)
        # Power
        self.motor.ChangeDutyCycle(abs(power))

    def GPIOcleanup(self):
        IO.cleanup()
