cmake_minimum_required(VERSION 3.0.2)
project(heatmap_util)

find_package(catkin REQUIRED)
find_package(rospy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/heatmap.py
  scripts/path_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
