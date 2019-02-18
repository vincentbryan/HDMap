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
        nav_msgs
        )

find_package( PCL REQUIRED)

add_message_files(
        FILES
        msg_signal.msg msg_signal_list.msg msg_map_cmd.msg msg_route_region.msg msg_route_info.msg
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

AUX_SOURCE_DIRECTORY(src/Type TYPE_SRC)
AUX_SOURCE_DIRECTORY(src/Math MATH_SRC)

add_library(TYPE_LIB STATIC
        ${TYPE_SRC}
        )

add_library(MATH_LIB STATIC
        ${MATH_SRC}
        )

add_executable(MapAgent
        src/MapAgent.cpp
        src/Tool/Client.cpp
        src/Tool/Sender.cpp
        )

add_executable(MapService
        src/MapService.cpp
        src/Tool/Sender.cpp
        src/Tool/Planner.cpp
        src/Tool/Resource.cpp
        )

add_executable(MapGenerator
        src/Tool/Sender.cpp
        src/MapGenerator_for_show.cpp
        )

add_executable(MapCommand
        src/MapCommand.cpp
        src/Tool/Sender.cpp
        )


add_executable(MapViewer
        src/MapViewer.cpp
        src/Tool/Sender.cpp
        )

add_executable(MapTest
        src/MapTest.cpp
        src/Tool/Sender.cpp
        )

add_executable(ViewDetectRegion
        src/Tool/ViewDetectRegion.cpp
        src/Tool/Sender.cpp
        )

add_executable(RepubPoint src/Tool/Republish.cpp)

add_executable(FakeMapCommand src/FakeMapCmd.cpp)

target_link_libraries(MapService ${catkin_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(MapGenerator ${catkin_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(MapAgent ${catkin_LIBRARIES} ${PCL_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(MapCommand ${catkin_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(MapViewer ${catkin_LIBRARIES} ${PCL_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(MapTest ${catkin_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(ViewDetectRegion ${catkin_LIBRARIES} ${PCL_LIBRARIES} TYPE_LIB MATH_LIB)
target_link_libraries(RepubPoint ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
target_link_libraries(FakeMapCommand ${catkin_LIBRARIES} )
