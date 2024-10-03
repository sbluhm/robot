import RPi.GPIO as IO
import time

IO.setwarnings(False)
IO.setmode (IO.BCM)

#Ready LED
LED=27
IO.setupLED, IO.OUT)
IO.output(LED, False)

def systemready():
    IO.output(LED, True)

def waiting():
    IO.output(LED, True)
    time.sleep(0.5)
    IO.output(LED, False)
    time.sleep(0.5)    
