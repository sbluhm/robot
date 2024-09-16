import pygame
# https://pinout.xyz/pinout/pin37_gpio26/
import time

import motorfunctions as motor

maxspeed = 20

pygame.init()

def main():
    # This dict can be left as-is, since pygame will generate a
    # pygame.JOYDEVICEADDED event for every joystick connected
    # at the start of the program.
    joysticks = {}

    axes_memory = {}
    button_memory = {}
    hat_memory = {}
    joystick_count = 0

    done = False
    while not done:
        # Event processing step.
        # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done so we exit this loop.

            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
                if event.button == 0:
                    joystick = joysticks[event.instance_id]
                    if joystick.rumble(0, 0.7, 500):
                        print(f"Rumble effect played on joystick {event.instance_id}")

            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

            # Get count of joysticks.
            if joystick_count != pygame.joystick.get_count():
                joystick_count = pygame.joystick.get_count()
        
                print( f"Number of joysticks: {joystick_count}")

                # For each joystick:
                for joystick in joysticks.values():
                    jid = joystick.get_instance_id()

                    print(f"Joystick {jid}")

                    # Get the name from the OS for the controller/joystick.
                    name = joystick.get_name()
                    print(f"Joystick name: {name}")

                    guid = joystick.get_guid()
                    print(f"GUID: {guid}")

                    power_level = joystick.get_power_level()
                    print(f"Joystick's power level: {power_level}")

                    # Usually axis run in pairs, up/down for one, and left/right for
                    # the other. Triggers count as axes.
                    axes = joystick.get_numaxes()
                    print(f"Number of axes: {axes}")

                    buttons = joystick.get_numbuttons()
                    print(f"Number of buttons: {buttons}")

                    hats = joystick.get_numhats()
                    print(f"Number of hats: {hats}")

#            for i in range(axes):
#                axis = joystick.get_axis(i)
#                if i not in axes_memory or axes_memory[i] != axis:
#                    print(f"Axis {i} value: {axis:>6.3f}")
#                    axes_memory[i] = axis
            speedvalue = joystick.get_axis(1)*-1
            if speedvalue > 0:
                motor.forward(speedvalue*maxspeed)
            elif speedvalue < 0:
                motor.reverse(speedvalue*-1*maxspeed)
            else:
                motor.idle()

            for i in range(buttons):
                button = joystick.get_button(i)
                if i not in button_memory or button_memory[i] != button:
                    print(f"Button {i:>2} value: {button}")
                    button_memory[i] = button

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            for i in range(hats):
                hat = joystick.get_hat(i)
                if i not in hat_memory or hat_memory[i] != hat:
                    print(f"Hat {i} value: {str(hat)}")
                    hat_memory[i] = hat


if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()
