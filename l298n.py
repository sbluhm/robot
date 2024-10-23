import RPi.GPIO as IO        
import time                        
IO.setwarnings(False)          
IO.setmode (IO.BCM)         

IO.setup(8,IO.OUT)        


p = IO.PWM(8,50000)
#IN1
IO.setup(7, IO.OUT)
IO.output(7, False)
#IN2
IO.setup(1, IO.OUT)
IO.output(1, False)


p.start(0)
while 1:                              
    for x in range (00,30,5):                      
        p.ChangeDutyCycle(x)               
        print(x)
        time.sleep(5)  
