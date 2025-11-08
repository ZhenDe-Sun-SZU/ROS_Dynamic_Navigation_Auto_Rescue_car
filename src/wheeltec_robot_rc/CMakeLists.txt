cmake_minimum_required(VERSION 2.8.3)
project(wheeltec_robot_rc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    joy
    std_msgs
    message_generation
	)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
	FILES
	pointss.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
  DEPENDS
)



