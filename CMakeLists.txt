cmake_minimum_required(VERSION 3.0.2)
project(hestia)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_runtime
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

find_package(yaml-cpp REQUIRED)

## Set Qt automations
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

## Declare ROS messages and services
add_message_files(
  FILES
  BushFire.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

## Declare a C++ executable
qt5_wrap_cpp(MOC_FILES include/hestia/user_interface.h)

add_executable(ui_node src/user_interface.cpp src/user_interface_node.cpp ${MOC_FILES})

add_executable(bushland_node src/bushland.cpp src/bushland_node.cpp)

add_executable(hestia_node src/Hestia.cpp src/hestia_node.cpp)

add_executable(apriltag_detector_node src/apriltag_detector.cpp src/apriltag_detector_node.cpp)

add_dependencies(ui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(apriltag_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ui_node
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

target_link_libraries(bushland_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
target_link_libraries(hestia_node
  ${catkin_LIBRARIES}
)

target_link_libraries(apriltag_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
)