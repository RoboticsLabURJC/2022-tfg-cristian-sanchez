cmake_minimum_required(VERSION 3.0.2)
project(teleop)

find_package(catkin REQUIRED)
find_package(rospy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_generation REQUIRED)

add_message_files(
  FILES
  Px4Cmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/px4_controller.py
  scripts/radio_control.py
  scripts/c2c_control.py
  scripts/manual_algorithm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
