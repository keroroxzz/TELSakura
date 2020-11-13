cmake_minimum_required(VERSION 2.8.3)
project(sakura_hardware)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  obstacle_msg.msg
)

add_service_files(
  FILES
  ObsDetect_srv.srv
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
