import RPi.GPIO as IO        
import time                        
IO.setwarnings(False)          
IO.setmode (IO.BCM)         
IO.setup(12,IO.OUT)
IO.setup(13,IO.OUT)
#Left Reverse Pin
IO.setup(5, IO.OUT)
#Left Reverse HIGH
IO.output(5, False)
#Right Reverse Pin
IO.setup(6, IO.OUT)
#Right Reverse HIGH
#IO.output(6, False)

o = IO.PWM(12,1500)
p = IO.PWM(13,1500)         
o.start(0)
p.start(0)
while 1:                              
    for x in range (50,100,10):                      
        o.ChangeDutyCycle(x)
        p.ChangeDutyCycle(x)
        print(x)
        time.sleep(5)  
