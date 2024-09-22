#!/usr/bin/env python
# https://pinout.xyz/pinout/pin37_gpio26/

# Gamepad Control
import Gamepad
# Keyboard Control
import curses

import time

import motorfunctions as motor

maxspeed = 50

def main():
# Define Gamepad
    gamepadType = Gamepad.PS4
    joystickSpeed = 'LEFT-Y'
    joystickSteering = 'L2'

# Define Keyboard
    # get the curses screen window
    screen = curses.initscr()
    # turn off input echoing
    curses.noecho()
    # respond to keys immediately (don't wait for enter)
    curses.cbreak()
    # map arrow keys to special values
    screen.keypad(True)


# Set some initial state
    speed = 0.0
    steering = 0.0

    done = False
    while not done:
        char = screen.getch()
        if char == ord('q'):
            curses.nocbreak()
            screen.keypad(0)
            curses.echo()
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            screen.addstr(0, 0, 'Clockwise         ')
            speed, steering = 0, 1
            curses.flushinp()
        elif char == curses.KEY_LEFT:
            screen.addstr(0, 0, 'Counter Clockwise ')
            speed, steering = 0, -1
            curses.flushinp()
        elif char == curses.KEY_UP:
            screen.addstr(0, 0, 'Forward           ')
            speed, steering = 20, 0
            curses.flushinp()
        elif char == curses.KEY_DOWN:
            screen.addstr(0, 0, 'Reverse           ')
            speed, steering = -20, 0
            curses.flushinp()
        elif char == ord('s'):
            screen.addstr(0, 0, 'Braking           ')
            stop()
            curses.flushinp()
        else:
            try:
                if gamepad.isConnected():
#                   print("Gamepad Connected and functional")
                    speed = -gamepad.axis(joystickSpeed)
                    steering = gamepad.axis(joystickSteering)
                    screen.addstr(0, 0, f'Vector: {speed}, {steering}')
                else:
#                   print("Gamepad configured but disconnected")
                    gamepad.disconnect()
            except:
                speed = 0.0
                steering = 0.0
#               print("gamepad not functional")
                if  Gamepad.available():
#                   print("Configuring gamepad")
                    gamepad = gamepadType()
                    gamepad.startBackgroundUpdates()
                    speed = -gamepad.axis(joystickSpeed)
                    steering = gamepad.axis(joystickSteering)


        motor.vdrive(speed,steering, maxspeed)

#        time.sleep(0.5)
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()

if __name__ == "__main__":
    main()
