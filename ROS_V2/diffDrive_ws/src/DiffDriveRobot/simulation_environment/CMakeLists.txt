cmake_minimum_required(VERSION 3.0.2)
project(simulation_environment)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    gazebo_ros)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy
  # other arguments like INCLUDE_DIRS or LIBRARIES can be added here
)


link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${src}
    ${GAZEBO_INCLUDE_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")