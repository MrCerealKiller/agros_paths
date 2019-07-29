cmake_minimum_required(VERSION 3.0.0)
project(agros_paths)

find_package(catkin REQUIRED COMPONENTS
  geodesy
  geographic_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
  unique_id
  uuid_msgs
)

catkin_python_setup()

###################
## Catkin Config ##
###################

catkin_package(
  CATKIN_DEPENDS
  geodesy
  geographic_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
  unique_id
  uuid_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/generate.py
  scripts/follow.py
  scripts/find_zone.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
