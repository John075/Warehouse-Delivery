cmake_minimum_required(VERSION 3.0.2)
project(warehouse_delivery_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

add_message_files(
        FILES
        DeliveryStatus.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS rospy)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        src/run_web_bridge.py   # Path to your script
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # Installation destination
)