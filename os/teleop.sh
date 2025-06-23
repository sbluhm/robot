#!/bin/bash

# Start teleop in the background
ros2 run bluhmbot teleop.py &
PID=$!

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/cmd_vel_teleop -p frame_id:=base_link 
ros2 launch teleop_twist_joy teleop-launch.py config_filepath:='/root/robot/ros2/src/bluhmbot/config/joystick.yaml' joy_vel:='/cmd_vel_teleop' publish_stamped_twist:='true'

echo "Press CTRL+C again to stop teleop in the background"

# Define a cleanup function
cleanup() {
    echo "Stopping programs..."
    kill $PID1 $PID2
    wait $PID1 $PID2 2>/dev/null
    echo "Programs stopped."
    exit 0
}

# Trap Ctrl+C (SIGINT) and call cleanup
trap cleanup SIGINT

# Wait for both programs to finish
wait

