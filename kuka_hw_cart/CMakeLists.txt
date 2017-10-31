cmake_minimum_required(VERSION 2.8.3)
project(kuka_hw_cart)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  kuka_ros_open_comm
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kuka_hardware_interface
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs
    kuka_ros_open_comm
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(kuka_hw_interface_cart
  src/kuka_hardware_interface_cart.cpp
)

target_link_libraries(kuka_hw_interface_cart
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_executable(kuka_hw_interface_node_cart
  src/kuka_hardware_interface_cart_main_node.cpp
)

target_link_libraries(kuka_hw_interface_node_cart
  kuka_hw_interface_cart
)

install(
  TARGETS
    kuka_hw_interface_cart
    kuka_hw_interface_node_cart
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
