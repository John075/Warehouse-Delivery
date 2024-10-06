#!/bin/bash
source ../devel/setup.bash
xvfb-run -s "-screen 0 1280x1024x24" roslaunch hector_moveit_gazebo orchyard_navigation_headless.launch
