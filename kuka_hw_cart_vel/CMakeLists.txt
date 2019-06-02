cmake_minimum_required(VERSION 2.8.3)
project(kuka_hw_cart_vel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  kuka_ros_open_comm
  realtime_tools
  roscpp
  std_msgs
  sensor_msgs
  tf
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS controller_manager hardware_interface kuka_ros_open_comm realtime_tools roscpp std_msgs
  #DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kuka_hw_cart_vel_main src/kuka_hw_cart_vel.cpp src/kuka_hw_cart_vel_main_node.cpp)
target_link_libraries(kuka_hw_cart_vel_main ${catkin_LIBRARIES})

add_executable(squareTrajectory src/squareTrajectory.cpp)
target_link_libraries(squareTrajectory ${catkin_LIBRARIES})

add_executable(kukaSinControl src/kukaSinControl.cpp)
target_link_libraries(kukaSinControl ${catkin_LIBRARIES})

add_executable(kukaCircleControl src/kukaCircleControl.cpp)
target_link_libraries(kukaCircleControl ${catkin_LIBRARIES})

add_executable(kukaHelisControl src/kukaHelisControl.cpp)
target_link_libraries(kukaHelisControl ${catkin_LIBRARIES})

add_executable(kuka_robot_state_publisher src/kuka_robot_state_publisher.cpp)
target_link_libraries(kuka_robot_state_publisher ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
