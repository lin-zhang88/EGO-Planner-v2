cmake_minimum_required(VERSION 3.0.2)
project(ros_to_dynamodb)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    sensor_msgs 
    nav_msgs 
    geometry_msgs 
    std_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/ros_to_dynamodb_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

