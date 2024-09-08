import curses
import time

import RPi.GPIO as IO        

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

def forward(time=0.5, speed=50):
    #Left Reverse HIGH
    IO.output(5, False)
    #Right Reverse HIGH
    IO.output(6, True)
    o.ChangeDutyCycle(speed)
    p.ChangeDutyCycle(speed)
    time.sleep(time)

def reverse(time=0.5, speed=50):
    #Left Reverse HIGH
    IO.output(5, True)
    #Right Reverse HIGH
    IO.output(6, False)
    o.ChangeDutyCycle(speed)
    p.ChangeDutyCycle(speed)
    time.sleep(time)

def idle():
    speed=0
    o.ChangeDutyCycle(speed)
    p.ChangeDutyCycle(speed)


o = IO.PWM(12,1500)
p = IO.PWM(13,1500)         
o.start(0)
p.start(0)



# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)
i=0
try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            screen.addstr(0, 0, 'right')
            curses.flushinp()
        elif char == curses.KEY_LEFT:
            screen.addstr(0, 0, 'left ')
            curses.flushinp()
        elif char == curses.KEY_UP:
            forward()
            screen.addstr(0, 0, f'Forward ')
            curses.flushinp()
        elif char == curses.KEY_DOWN:
            reverse()
            screen.addstr(0, 0, 'down     ')
            curses.flushinp()
        else:
            idle()
finally:
    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()
