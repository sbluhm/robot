import RPi.GPIO as IO
#import time

IO.setwarnings(False)
IO.setmode (IO.BCM)
#Right PWM Pin
IO.setup(12,IO.OUT)
#Left PWM Pin
IO.setup(13,IO.OUT)
#Left Reverse Pin
IO.setup(5, IO.OUT)
#Left brake pin
IO.setup(26, IO.OUT)

#Right Reverse Pin
IO.setup(6, IO.OUT)
#Right brake pin
IO.setup(25, IO.OUT)

right = IO.PWM(12,10000)
left = IO.PWM(13,10000)
right.start(0)
left.start(0)

class MotorDriver:
    def __init__(self, max_speed=100, scale_speed=100):
        """
        Init communication, set default settings, ...
        """
        self.max_speed = max_speed
        self.scale_speed = scale_speed
        self.wheelbase = 0.46
        self.current_speed = 0
        self.voltage = 12
        self.temperature = 47
    def set_speed(self, speed):
        """
        Give a speed that the motor will try to reach.
        """
        if speed <= self.max_speed:
            self.current_speed = speed
        else:
            self.current_speed = self.max_speed

    def stop(self):
        """
        Set speed to 0 and thus stop the motor
        """
        right.ChangeDutyCycle(0)
        left.ChangeDutyCycle(0)
        IO.output(25, True)
        IO.output(26, True)
        print("Engine Stopped")

    def get_speed(self):
        """
        Return current speed
        """
        return self.current_speed
    def get_status(self):
        """
        Get hardware information from the motor
        """
        return {
            'temperature': self.temperature,
            'voltage': self.voltage
        }
# Vector Drive
    def vdrive(self,y,x):
        scale = self.scale_speed
        if y == 0:         # voller einschlag falls nur links/rechts betätigt wird
            l = x*0.5
            r = -x*0.5
        else:
            if x >= 0:
                l = y
                r = y-x*y
            else:
                r = y
                l = y+x*y

        self.leftwheel(l*scale)
        self.rightwheel(r*scale)

    def twistdrive(self,linearx,angularz):
        l = linearx - angularz*self.wheelbase/2
        r = linearx + angularz*self.wheelbase/2

        self.leftwheel(l)
        self.rightwheel(r)

    def leftwheel(self, vector):
        # Release brakes
        IO.output(25, False)

        # Set direction
        if vector < 0:
            IO.output(6, True)
        else:
            IO.output(6, False)
        # Power
        left.ChangeDutyCycle(abs(vector))

    def rightwheel(awlf, vector):
        # Release brakes
        IO.output(26, False)

        # Set direction
        if vector < 0:
             IO.output(5, False)
        else:
             IO.output(5, True)
        # Power
        right.ChangeDutyCycle(abs(vector))

    def GPIOcleanup(self):
        IO.cleanup()
