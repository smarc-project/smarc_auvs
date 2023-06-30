cmake_minimum_required(VERSION 2.8.3)
project(captain_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  lolo_msgs
  smarc_msgs
  geographic_msgs
  genmsg
  ixblue_ins_msgs
)

## Messages
#add_message_files(
#  FILES
#  #CaptainStatus.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs sensor_msgs lolo_msgs smarc_msgs
  INCLUDE_DIRS include
  LIBRARIES other_stuff
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(other_stuff
  src/CaptainInterFace/CaptainInterFace.cpp
  #src/TcpInterFace/TcpInterFace.cpp
  src/UDPInterface/UDPInterface.cpp
  src/RosInterFace/RosInterFace.cpp
  src/RosInterFace/RosInterFace_ros_callbacks.cpp
  src/RosInterFace/RosInterFace_captain_callbacks.cpp
)

add_executable(interface src/main.cpp)

add_dependencies(other_stuff ${catkin_EXPORTED_TARGETS})
add_dependencies(interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  interface
  other_stuff
  ${catkin_LIBRARIES}
)

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/lolo_translator.py
  scripts/menu_input.py
  scripts/menu_output.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(TARGETS interface other_stuff
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
