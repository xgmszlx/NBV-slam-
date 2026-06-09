cmake_minimum_required(VERSION 3.0.2)
project(hnbv_semantics)

find_package(catkin REQUIRED COMPONENTS hnbv_msgs rospy sensor_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/semantic_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

