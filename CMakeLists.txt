cmake_minimum_required(VERSION 2.8.3)
project(waypoint_generator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_python_setup()
catkin_package()
