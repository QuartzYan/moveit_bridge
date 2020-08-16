cmake_minimum_required(VERSION 2.8.3)
project(moveit_bridge)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  moveit_msgs
  rospy
  std_msgs
)
catkin_python_setup()

add_message_files(
  FILES
  pose.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package()

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

