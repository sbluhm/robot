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
    wmax = 2  # maximale Leistung (beide motoren auf vollgas)
    g = 2 # gewichtung des verhältnisses bewegung/einlenken

    w = y * wmax   # die leistung die beide motoren liefern sollen
    if y == 0:         # voller einschlag falls nur links/rechts betätigt wird
        r = -x
        l = x
    else:
        t = w / (g+abs(x))
        if x >= 0:
            r = t
            l = w - t
        else:
            r = w - t
            l = t
    if l > 1:
        r = r / l
        l = 1
    elif r > 1:
        l = l / r
        r = 1
    leftwheel(l*scale)
    rightwheel(r*scale)
   

def leftwheel(vector):
    # Release brakes

    # Set direction
    IO.output(25, False)
    if vector > 0:
        IO.output(5, True)
    else:
        IO.output(5, False)

    # Power
    left.ChangeDutyCycle(abs(vector))

    print(f"Left Motor ({vector} %)")

def rightwheel(vector):
    # Release brakes

    # Set direction
    IO.output(26, False)
    if vector > 0:
        IO.output(6, False)
    else:
        IO.output(6, True)

    # Power
    right.ChangeDutyCycle(abs(vector))

    print(f"Right Motor ({vector} %)")


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

