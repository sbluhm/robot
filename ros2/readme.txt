# Source workspace
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Build packages
colcon build --symlink

# Source the overlay
source install/local_setup.bash

# Run a specific package

# Launch the robot
