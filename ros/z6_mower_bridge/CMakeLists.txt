cmake_minimum_required(VERSION 3.0.2)
project(z6_mower_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/z6_mower_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
