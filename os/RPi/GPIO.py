from time import sleep
from threading import Thread

RISING=True

_current_dutycycle=0

def _dummy_interrupt(callback, pin):
    global _current_dutycycle
    _MOTOR_ROC = 32.2418230613342
    _MOTOR_SHIFT = -0.155436252405753
    _MIN_POWER = 4.366

    while True:
        if _current_dutycycle >= _MIN_POWER:
            sleep(1/(( _current_dutycycle / _MOTOR_ROC - _MOTOR_SHIFT) * 90))
            callback(pin)
        else:
            sleep(0.1)

def BCM():
    return True

def OUT():
    return True

def IN():
    return True

def setwarnings(dummy):
    return True

def setmode (dummy):
    return True

def setup(dummy1, dummy2):
    return True

def output(dummy1, dummy2):
    return True

def add_event_detect(pin, edge,callback, bouncetime=10):
    dummy_interrupt_thread = Thread(target=_dummy_interrupt, args=(callback,pin,), daemon = True)
    # Start the thread.
    dummy_interrupt_thread.start()

class PWM():
    def __init__(self,dummy1, dummy2):
        return None
    def start(self, dummy):
        return True
    def ChangeDutyCycle(self, dutycycle):
        global _current_dutycycle
        _current_dutycycle=dutycycle
        return True

