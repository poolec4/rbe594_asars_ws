# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(asars_global_planner)

## Find catkin and any catkin packages
find_package(
        catkin
        REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        actionlib
        genmsg
        actionlib_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
)

## Declare ROS messages and services
add_service_files(DIRECTORY srv FILES GenerateVisitingOrder.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES geometry_msgs nav_msgs)

## Declare a catkin package
catkin_package( INCLUDE_DIRS include/${PROJECT_NAME}/)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

## Build service
add_executable(visiting_order_generator_node src/visiting_order_generator_node.cpp)
target_link_libraries(visiting_order_generator_node ${catkin_LIBRARIES})
