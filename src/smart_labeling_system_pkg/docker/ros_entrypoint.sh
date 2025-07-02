#!/bin/bash
set -e

# Source the ROS 2 Humble setup file to make ROS commands available
source /opt/ros/humble/setup.bash

# Source your workspace's setup file to make your packages and nodes available
source /ros2_ws/install/setup.bash

# Execute the command passed into the container (e.g., the 'ros2 launch...' from CMD)
exec "$@"