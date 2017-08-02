cmake_minimum_required(VERSION 2.8.3)
project(guide_face_recog)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs face_recognition sound_play)

set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(guide_face_recog guide_face_recog.cpp)
target_link_libraries(guide_face_recog ${catkin_LIBRARIES})
