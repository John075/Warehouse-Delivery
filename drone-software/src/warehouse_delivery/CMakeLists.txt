cmake_minimum_required(VERSION 2.8.3)
project(warehouse_delivery)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    hector_uav_msgs
    hector_moveit_actions
    actionlib
    gazebo_ros_link_attacher
    rospy
    message_generation
)

add_message_files(
        FILES
        MoveAction.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs hector_uav_msgs hector_moveit_actions actionlib gazebo_ros_link_attacher)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(delivery_node src/Explorer.cpp src/explorer_node.cpp)
target_compile_options(delivery_node PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})
target_link_libraries(delivery_node ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(delivery_node hector_uav_msgs_generate_messages hector_moveit_actions_generate_messages gazebo_ros_link_attacher)

catkin_install_python(PROGRAMS
        test/test_explorer.py   # Path to your script
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # Installation destination
)