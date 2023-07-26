cmake_minimum_required(VERSION 2.8.3)
project(cbgl)

add_compile_options(-std=c++11 -Ofast)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  message_generation
  pcl_ros
)

find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

find_package(CGAL REQUIRED)


catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  nav_msgs
  message_runtime
  INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  /usr/include/eigen3/
)

link_directories(${csm_LIBRARY_DIRS})

add_library(cbgl_utils_lib
  src/utils/occupancy_grid_utils/coordinate_conversions.cpp
  src/utils/occupancy_grid_utils/ray_tracer.cpp
)

target_link_libraries(cbgl_utils_lib
  ${catkin_LIBRARIES}
)

add_subdirectory(src/cbgl_node)
