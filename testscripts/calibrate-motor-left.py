import RPi.GPIO as IO
import time
IO.setwarnings(False)
IO.setmode (IO.BCM)
IO.setup(13,IO.OUT)
p = IO.PWM(13,1000)
#Reverse Pin
IO.setup(6, IO.OUT)
# Reverse HIGH
IO.output(6, False)
# Speedpulse
speed_pulse_pin=16
IO.setup(speed_pulse_pin, IO.IN)
tick_counter = 1000000000.0
__direction = 1

def speed_pulse_callback(interrupt_pin):
        global tick_counter
        tick_counter = tick_counter + __direction




IO.add_event_detect(speed_pulse_pin, IO.RISING,
            callback=speed_pulse_callback, bouncetime=5)

p.start(0)


for x in range (10,255,10):
    p.ChangeDutyCycle(int(x/255*100))
    time.sleep(5)
    tick_counter=0
    time.sleep(65)
    print( f"Power: {int(x/255*100)} - ticks: {tick_counter} -- Speed: {tick_counter/90 / 65}")

