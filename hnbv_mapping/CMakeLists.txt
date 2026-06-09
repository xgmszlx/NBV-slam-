cmake_minimum_required(VERSION 3.0.2)
project(hnbv_mapping)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp sensor_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(occupancy_projection_node src/occupancy_projection_node.cpp)
target_link_libraries(occupancy_projection_node ${catkin_LIBRARIES})

add_executable(masked_pointcloud_node src/masked_pointcloud_node.cpp)
target_link_libraries(masked_pointcloud_node ${catkin_LIBRARIES})

install(TARGETS occupancy_projection_node masked_pointcloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

