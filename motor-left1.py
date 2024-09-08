import RPi.GPIO as IO        
import time                        
IO.setwarnings(False)          
IO.setmode (IO.BCM)         
IO.setup(12,IO.OUT)        
p = IO.PWM(12,1000)         
p.start(0)
while 1:                              
    for x in range (50,100,10):                      
        p.ChangeDutyCycle(x)               
        print(x)
        time.sleep(5)  
