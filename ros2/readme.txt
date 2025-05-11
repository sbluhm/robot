# Source workspace
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install -i --from-path src --rosdistro jazzy -y

# Build packages
colcon build --symlink

# Source the overlay
source install/local_setup.bash

# Run a specific package
ros2 start 

# Launch the robot
ros2 launch bluhmbot robot_launch.py

# Instructions for profiling
https://robotics.stackexchange.com/questions/99990/how-to-run-code-profiler-for-your-ros2-nodes
