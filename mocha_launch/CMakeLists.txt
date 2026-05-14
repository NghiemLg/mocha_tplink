cmake_minimum_required(VERSION 2.8.3)
project(mocha_launch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometric_controller
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mocha_launch
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY
  launch
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/delay_done.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/iris.launch)
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/jackal.launch)
endif()