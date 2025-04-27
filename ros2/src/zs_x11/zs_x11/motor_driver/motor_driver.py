import RPi.GPIO as IO
import time

MAX_POWER_VALUE_SCALE = 100.0/255.0

IO.setwarnings(False)
IO.setmode (IO.BCM)

class MotorDriver:
    def __init__(self, pwm_pin=13, reverse_pin=6, brake_pin=26, speed_pulse_pin=19, inverse=False):
        """
        Init communication, set default settings, ...
        """
        self.tick_counter = 0
        self.last_tick_time = time.time()
        self.inverse_multiplier = 1
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
        IO.add_event_detect(speed_pulse_pin, IO.FALLING, 
            callback=self.speed_pulse_callback, bouncetime=10)
        self.motor = IO.PWM(self.pwm_pin,10000)
        self.motor.start(0)

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        self.motor.ChangeDutyCycle(0)
        IO.output(self.brake_pin, True)

    def power_to_ticks_per_second(self, power):
        # 90 ticks per revolution
        if power == 0:
            tps = 0
        else:
            tps = (95 + power/2) * 90 / 60
        return tps

    def count_ticks(self, power):
        now = time.time()
        dt = now - self.last_tick_time
        self.last_tick_time = now
        self.tick_counter += self.power_to_ticks_per_second(power) * dt

    def speed_pulse_callback(self):
        self.tick_counter += 1

    def wheel(self, power):
        # Release brakes
        IO.output(self.brake_pin, False)

        # Set direction and scale to 255
        power = self.inverse_multiplier * power * MAX_POWER_VALUE_SCALE
        if power < 0:
            IO.output(self.reverse_pin, True)
        else:
            IO.output(self.reverse_pin, False)
        # Power
        self.motor.ChangeDutyCycle(abs(power))
        self.count_ticks(power)

    def GPIOcleanup(self):
        IO.cleanup()
