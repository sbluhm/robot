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

    # Set some initial state
    speed = 0.0
    steering = 0.0

    done = False
    while not done:
        try:
            if gamepad.isConnected():
#                print("Gamepad Connected and functional")
                speed = -gamepad.axis(joystickSpeed)
            # Steering control (not inverted)
                steering = gamepad.axis(joystickSteering)
            else:
#                print("Gamepad configured but disconnected")
                gamepad.disconnect()
        except:
            speed = 0.0
            steering = 0.0
#            print("gamepad not functional")
            if  Gamepad.available():
#                print("Configuring gamepad")
                gamepad = gamepadType()
                gamepad.startBackgroundUpdates()
                speed = -gamepad.axis(joystickSpeed)
                steering = gamepad.axis(joystickSteering)


        motor.vdrive(speed,steering)

#        time.sleep(0.5)

if __name__ == "__main__":
    main()
