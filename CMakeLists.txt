cmake_minimum_required(VERSION 3.5)
project(HDMap)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rosbag
		std_msgs
		geometry_msgs
		message_generation

		tf
		tf_conversions
)

find_package( PCL REQUIRED)

add_message_files(
FILES
msg_signal.msg msg_signal_list.msg msg_map_cmd.msg msg_route_region.msg
)
add_service_files(
FILES srv_route.srv srv_map_cmd.srv srv_map_data.srv
)

generate_messages(
DEPENDENCIES std_msgs geometry_msgs # Or other packages containing msgs
)

catkin_package(
  	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


AUX_SOURCE_DIRECTORY(src/Type TYPES)
AUX_SOURCE_DIRECTORY(src/Math MATH)


add_executable(MapAgent
		${TYPES}
		${MATH}
		src/MapAgent.cpp
		src/Tool/Client.cpp
		src/Tool/Sender.cpp
)

add_executable(MapService
		${TYPES}
		${MATH}
		src/MapService.cpp
		src/Tool/Sender.cpp
		src/Tool/Planner.cpp
		src/Tool/Resource.cpp
)

add_executable(MapGenerator
		${TYPES}
		${MATH}
		src/Tool/Sender.cpp
		src/MapGenerator.cpp
)

add_executable(MapCommand
		${TYPES}
		${MATH}
		src/MapCommand.cpp
		src/Tool/Sender.cpp
)

add_executable(ViewGPS
		${TYPES}
		${MATH}
		src/Tool/ViewGPS.cpp
		src/Tool/Sender.cpp
		)

add_executable(MapViewer
		${TYPES}
		${MATH}
		src/MapViewer.cpp
		src/Tool/Sender.cpp
)

add_executable(MapTest
		${TYPES}
		${MATH}
		src/MapTest.cpp
		src/Tool/Sender.cpp
)

add_executable(ViewDetectRegion
		${TYPES}
		${MATH}
		src/Tool/ViewDetectRegion.cpp
		src/Tool/Sender.cpp
)

add_executable(RepubPoint src/Tool/Republish.cpp)

target_link_libraries(MapService
${catkin_LIBRARIES}
)

target_link_libraries(ViewGPS
${catkin_LIBRARIES}
)

target_link_libraries(MapGenerator
${catkin_LIBRARIES}
)

target_link_libraries(MapAgent
${catkin_LIBRARIES}
)

target_link_libraries(MapCommand
${catkin_LIBRARIES}
)

target_link_libraries(MapViewer
${catkin_LIBRARIES}
)

target_link_libraries(MapTest
${catkin_LIBRARIES}
)

target_link_libraries(ViewDetectRegion
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)

target_link_libraries(RepubPoint ${catkin_LIBRARIES} ${PCL_LIBRARIES})
