# https://pinout.xyz/pinout/pin37_gpio26/
import curses
import time

import RPi.GPIO as IO        

IO.setwarnings(False)          
IO.setmode (IO.BCM)         
#Right PWM Pin
IO.setup(12,IO.OUT)
#Left PWM Pin
IO.setup(13,IO.OUT)
#Left Reverse Pin
IO.setup(5, IO.OUT)
#Left brake pin
IO.setup(26, IO.OUT)

#Right Reverse Pin
IO.setup(6, IO.OUT)
#Right brake pin
IO.setup(25, IO.OUT)


def forward(speed=20, runtime=0.5):
    #Left Reverse HIGH
    IO.output(5, True)
    #Right Reverse HIGH
    IO.output(6, False)
    #Release brakes
    IO.output(25, False)
    IO.output(26, False)

    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)
    time.sleep(runtime)

def reverse(speed=20, runtime=0.5):
    #Left Reverse HIGH
    IO.output(5, False)
    #Right Reverse HIGH
    IO.output(6, True)
    IO.output(25, False)
    IO.output(26, False)

    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)
    time.sleep(runtime)

def idle():
    IO.output(25, False)
    IO.output(26, False)
    speed=0
    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)

def stop():
    right.ChangeDutyCycle(0)
    left.ChangeDutyCycle(0)
    IO.output(25, True)
    IO.output(26, True) 


right = IO.PWM(12,1500)
left = IO.PWM(13,1500)         
right.start(0)
left.start(0)



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
            screen.addstr(0, 0, 'Forward     ')
            curses.flushinp()
        elif char == curses.KEY_DOWN:
            reverse(100)
            screen.addstr(0, 0, 'Reverse     ')
            curses.flushinp()
        elif char == ord('s'):
            screen.addstr(0, 0, 'Braking     ')
            stop()
            curses.flushinp()
            time.sleep(5)
        idle()
finally:
    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()
