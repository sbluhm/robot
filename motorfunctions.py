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

# Vector Drive
def vdrive(y,x,scale = 20):
#    print(f"Input Vorwärts: {y}, Seitwärts: {x}")
    if y == 0:         # voller einschlag falls nur links/rechts betätigt wird
        l = x
        r = -x
    else:
        if x >= 0:
            l = y
            r = y-x*y
        else:
            r = y
            l = y+x*y

    leftwheel(l*scale)
    rightwheel(r*scale)
#    time.sleep(0.5) 

def leftwheel(vector):
    # Release brakes
    IO.output(25, False)

    # Set direction
    if vector < 0:
        IO.output(6, True)
    else:
        IO.output(6, False)

    # Power
    left.ChangeDutyCycle(abs(vector))

#    print(f"Left Motor ({vector} %)")

def rightwheel(vector):
    # Release brakes
    IO.output(26, False)

    # Set direction
    if vector < 0:
        IO.output(5, False)
    else:
        IO.output(5, True)

    # Power
    right.ChangeDutyCycle(abs(vector))

#    print(f"Right Motor ({vector} %)")


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

