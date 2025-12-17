# Andromeda Proximity Filter

## Build and Run Instructions

Make sure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash

cd ~/ros_ws

colcon build --packages-select andromeda_proximity_filter

source install/setup.bash

ros2 launch andromeda_proximity_filter proximity_filter_launch.py

colcon test --packages-select andromeda_proximity_filter
colcon test-result --verbose


