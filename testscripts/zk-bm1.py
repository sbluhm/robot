import RPi.GPIO as IO        
import time                        
IO.setwarnings(False)          
IO.setmode (IO.BCM)         

PWM_FWD_PIN = 10
PWM_REV_PIN = 9

IO.setup(PWM_FWD_PIN,IO.OUT)
IO.setup(PWM_REV_PIN,IO.OUT)
p = IO.PWM(PWM_FWD_PIN,10000)
o = IO.PWM(PWM_REV_PIN,10000)


p.start(0)
o.start(0)
while 1:                              
    for x in range (00,30,5):                      
        p.ChangeDutyCycle(x)               
        print(x)
        time.sleep(5)  



