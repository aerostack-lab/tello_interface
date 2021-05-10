cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME tello_interface)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake

#add_definitions(-std=c++0x)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)
add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


set(DRIVER_TELLO_SOURCE_DIR
	src/source)
	
set(DRIVER_TELLO_INCLUDE_DIR
	src/include
	)

set(DRIVER_TELLO_HEADER_FILES
        ${DRIVER_TELLO_INCLUDE_DIR}/socket_tello.h
    )

set(DRIVER_TELLO_SOURCE_FILES
        ${DRIVER_TELLO_SOURCE_DIR}/socket_tello.cpp
        ${DRIVER_TELLO_SOURCE_DIR}/socket_tello_camera.cpp
        ${DRIVER_TELLO_SOURCE_DIR}/socket_tello_command.cpp
        ${DRIVER_TELLO_SOURCE_DIR}/socket_tello_state.cpp
    )

find_package(catkin REQUIRED
                COMPONENTS lib_cvgutils tf_conversions cv_bridge)


catkin_package(
#       INCLUDE_DIRS ${OKTODRIVER_INCLUDE_DIR}
        CATKIN_DEPENDS lib_cvgutils tf_conversions cv_bridge
  )


include_directories(${DRIVER_TELLO_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(tello_interface ${DRIVER_TELLO_SOURCE_FILES} ${DRIVER_TELLO_HEADER_FILES})
add_dependencies(tello_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_interface ${catkin_LIBRARIES} avcodec avutil swscale)

target_include_directories(tello_interface PUBLIC ${DRIVER_TELLO_INCLUDE_DIR})

set_target_properties(tello_interface PROPERTIES LINKER_LANGUAGE CXX)