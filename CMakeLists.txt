cmake_minimum_required(VERSION 3.0.2)
project(MP_DR_T18)
find_package(catkin REQUIRED COMPONENTS
  interiit22
  prius_description
  prius_msgs
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/camera.py scripts/autopilot.py scripts/control.py scripts/image_processing.py scripts/simple.py scripts/util.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)