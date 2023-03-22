cmake_minimum_required(VERSION 2.8.3)
project(intrepid_streaming_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

add_message_files(
   FILES
   CompressedUGVStream.msg
   UGVStream.msg
   CompressedMAXStream.msg
   MAXStream.msg
)

#add_service_files(
#  FILES
#)
#
#add_action_files(
#  FILES
#)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intrepid_streaming_msgs
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs sensor_msgs
#  DEPENDS system_lib
)
