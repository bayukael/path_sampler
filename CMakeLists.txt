cmake_minimum_required(VERSION 2.8.3)

project(path_sampler)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

# catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs rospy std_msgs
)

catkin_install_python(
  PROGRAMS scripts/path_sampler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

