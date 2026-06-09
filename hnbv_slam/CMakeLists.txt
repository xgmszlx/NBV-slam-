cmake_minimum_required(VERSION 3.0.2)
project(hnbv_slam)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS geometry_msgs hnbv_msgs roscpp std_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(slam_state_publisher src/slam_state_publisher.cc)
add_dependencies(slam_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_state_publisher ${catkin_LIBRARIES})

install(TARGETS slam_state_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

