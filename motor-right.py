import RPi.GPIO as IO        
import time                        
IO.setwarnings(False)          
IO.setmode (IO.BCM)         
IO.setup(13,IO.OUT)        
p = IO.PWM(13,1000)         
p.start(0)
while 1:                              
    for x in range (0,101,10):                      
        p.ChangeDutyCycle(x)               
        print(x)
        time.sleep(5)  
