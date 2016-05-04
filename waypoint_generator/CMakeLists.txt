cmake_minimum_required(VERSION 2.8.3)
project(waypoint_generator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_python_setup()
catkin_package()

install(PROGRAMS
  scripts/generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
