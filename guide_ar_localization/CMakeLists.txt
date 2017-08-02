cmake_minimum_required(VERSION 2.8.3)
project(guide_ar_localization)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs visualization_msgs sound_play)

set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(guide_ar_localization guide_ar_localization.cpp)
target_link_libraries(guide_ar_localization ${catkin_LIBRARIES})
