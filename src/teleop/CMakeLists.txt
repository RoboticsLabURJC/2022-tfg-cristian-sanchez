cmake_minimum_required(VERSION 3.0.2)
project(teleop)

find_package(catkin REQUIRED)
find_package(rospy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleop
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/px4_controller.py
  scripts/radio_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
