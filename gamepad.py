#!/usr/bin/env python
# https://pinout.xyz/pinout/pin37_gpio26/

import Gamepad
import time

import motorfunctions as motor

maxspeed = 20

def main():
    gamepadType = Gamepad.PS4
    joystickSpeed = 'LEFT-Y'
    joystickSteering = 'L2'

    # Wait for a connection
    if Gamepad.available():
        gamepad = gamepadType()
        print('Gamepad connected. Using Gamepad')
    else:
        print("Gamepad not found.")

    # Set some initial state
    speed = 0.0
    steering = 0.0

    # Start the background updating
    gamepad.startBackgroundUpdates()

    done = False
    while not done:
        speed = -gamepad.axis(joystickSpeed)
        # Steering control (not inverted)
        steering = gamepad.axis(joystickSteering)

        motor.vdrive(speed,steering)

#        if speed > 0:
#            motor.forward(speed*maxspeed)
#        elif speed < 0:
#            motor.reverse(speed*-1*maxspeed)
#        else:
#            motor.idle()
    time.sleep(0.1)

if __name__ == "__main__":
    main()
