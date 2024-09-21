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
    if not Gamepad.available():
        print('Please connect your gamepad...')
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print('Gamepad connected')

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

    time.sleep(0.1)

if __name__ == "__main__":
    main()
