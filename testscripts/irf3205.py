import RPi.GPIO as IO        
import time                        
IO.setwarnings(False)          
IO.setmode (IO.BCM)         

PWM_PIN = 11
DIR_PIN = 8

IO.setup(PWM_PIN,IO.OUT)        
p = IO.PWM(PWM_PIN,10000)


#Direction
IO.setup(DIR_PIN, IO.OUT)
IO.output(DIR_PIN, False)


p.start(0)
while 1:                              
    for x in range (00,30,5):                      
        p.ChangeDutyCycle(x)               
        print(x)
        time.sleep(5)  
