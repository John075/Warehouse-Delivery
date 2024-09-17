#!/bin/bash

# Source the ROS environment
source devel/setup.bash

# Publish to the Gazebo reset topic to reset the simulation
rosservice call /gazebo/reset_simulation "{}"

# Run the test script
roslaunch hector_moveit_exploration explore.launch
