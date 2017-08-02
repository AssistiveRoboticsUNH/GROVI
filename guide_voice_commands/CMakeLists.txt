cmake_minimum_required(VERSION 2.8.3)
project(guide_voice_commands)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sound_play move_base_msgs actionlib guide_navigation_feedback)

set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(guide_voice_commands guide_voice_commands.cpp landmark.cpp)
target_link_libraries(guide_voice_commands ${catkin_LIBRARIES})
