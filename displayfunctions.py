import RPi.GPIO as IO
import time

IO.setwarnings(False)
IO.setmode (IO.BCM)

#Ready LED
IO.setup(17, IO.OUT)
IO.output(5, False)

def systemready():
    IO.output(5, True)

def waiting():
    IO.output(5, True)
    time.sleep(0.5)
    IO.output(5, False)
    
