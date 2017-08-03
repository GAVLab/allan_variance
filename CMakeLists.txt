cmake_minimum_required(VERSION 2.8.3)
project(allan_variance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  rosbag
)

catkin_package()

#############
## Install ##
#############
install(
  DIRECTORY
  	scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)