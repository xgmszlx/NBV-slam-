cmake_minimum_required(VERSION 3.0.2)
project(hnbv_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hnbv_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(global_nbv_node src/global_nbv_node.cpp)
add_dependencies(global_nbv_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_nbv_node ${catkin_LIBRARIES})

add_executable(local_nbv_node src/local_nbv_node.cpp)
add_dependencies(local_nbv_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_nbv_node ${catkin_LIBRARIES})

install(TARGETS global_nbv_node local_nbv_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

