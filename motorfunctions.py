import RPi.GPIO as IO
import time

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

right = IO.PWM(12,1500)
left = IO.PWM(13,1500)
right.start(0)
left.start(0)


def forward(speed=20, runtime=0.5):
    #Left Reverse HIGH
    IO.output(5, True)
    #Right Reverse HIGH
    IO.output(6, False)
    #Release brakes
    IO.output(25, False)
    IO.output(26, False)

    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)
    print(f"Motor Forward ({speed} %)")

def reverse(speed=20, runtime=0.5):
    #Left Reverse HIGH
    IO.output(5, False)
    #Right Reverse HIGH
    IO.output(6, True)
    IO.output(25, False)
    IO.output(26, False)

    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)
    print(f"Motor Reverse ({speed} %)")

def cw(speed=50, runtime=0.5):
    #Left Reverse HIGH
    IO.output(5, False)
    #Right Reverse HIGH
    IO.output(6, False)
    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)
    time.sleep(runtime)

def ccw(speed=50, runtime=0.5):
    #Left Reverse HIGH
    IO.output(5, True)
    #Right Reverse HIGH
    IO.output(6, True)
    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)
    time.sleep(runtime)

def idle():
    IO.output(25, False)
    IO.output(26, False)
    speed=0
    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)

def stop():
    right.ChangeDutyCycle(0)
    left.ChangeDutyCycle(0)
    IO.output(25, True)
    IO.output(26, True)

