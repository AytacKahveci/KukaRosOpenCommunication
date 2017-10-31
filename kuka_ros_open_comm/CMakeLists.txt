cmake_minimum_required(VERSION 2.8.3)
project(kuka_ros_open_comm)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(include)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  )

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
