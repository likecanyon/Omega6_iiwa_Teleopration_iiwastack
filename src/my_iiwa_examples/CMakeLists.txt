cmake_minimum_required(VERSION 2.8.3)
project(my_iiwa_examples)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()


find_library(DHDLIB libdhd.so lib)
find_library(USB1LIB libusb-1.0.so)

include_directories( include ${catkin_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_executable(command_robot src/command_robot.cpp src/HapticDevice.cpp)
add_dependencies(command_robot iiwa_msgs_generate_messages_cpp)
target_link_libraries(command_robot ${DHDLIB}
${USB1LIB} ${catkin_LIBRARIES})



add_executable(setting_control_mode src/setting_control_mode.cpp src/HapticDevice.cpp)
add_dependencies(setting_control_mode iiwa_msgs_generate_messages_cpp)
target_link_libraries(setting_control_mode   ${DHDLIB}
${USB1LIB} ${catkin_LIBRARIES})