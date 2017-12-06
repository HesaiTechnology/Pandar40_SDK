cmake_minimum_required(VERSION 2.8.3)
project(pandora_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  pandar_msgs
)


#add_message_files(
#  DIRECTORY msg
#  FILES
#  PandarGps.msg
#)
#
#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS pandar_msgs std_msgs 
)

add_executable(pandora_sdk_test 
	main.c
)

INCLUDE_DIRECTORIES(
	/usr/local/include
)
link_directories(
	/usr/local/lib
)

target_link_libraries(pandora_sdk_test 
	pandoraSDK
	${catkin_LIBRARIES}
)