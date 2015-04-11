cmake_minimum_required(VERSION 2.8.3)
project(hexa_msgs)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
	FILES
	LegsJointsState.msg
	AnglesValues.msg
	AngleValue.msg
)

add_service_files(
	FILES
	LegsJointsStateSrv.srv
)

generate_messages(
	DEPENDENCIES std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	${catkin_INCLUDE_DIRS}
)