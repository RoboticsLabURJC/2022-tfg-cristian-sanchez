cmake_minimum_required(VERSION 3.0.2)
project(heatmap_util)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(rospy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  grid_map_core
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_ros
  grid_map_rviz_plugin
  grid_map_sdf
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters

  genmsg
  actionlib_msgs
  actionlib
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(octomap REQUIRED)

add_action_files(DIRECTORY action FILES RvizFriss.action DroneFriss.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

add_executable(heat src/heat.cpp)

target_link_libraries(
  heat
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS 
  scripts/path_marker.py
  scripts/rf_heatmap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
