cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME tello_interface_process)
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
set(DRIVER_TELLO_TEST_DIR
    src/test
)

set(DRIVER_TELLO_HEADER_FILES
        src/include/tello_camera_interface.h
        src/include/tello_command_interface.h
        src/include/tello_state_interface.h
    )

set(DRIVER_TELLO_SOURCE_FILES
    )

find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros mav_msgs robot_process aerostack_msgs lib_cvgutils tf_conversions cv_bridge tello_interface)


catkin_package(
#       INCLUDE_DIRS ${OKTODRIVER_INCLUDE_DIR}
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros mav_msgs robot_process aerostack_msgs lib_cvgutils tf_conversions tello_interface
  )


include_directories(${DRIVER_TELLO_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(tello_interface_process ${DRIVER_TELLO_SOURCE_FILES} ${DRIVER_TELLO_HEADER_FILES})
add_dependencies(tello_interface_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_interface_process ${catkin_LIBRARIES} avcodec avutil swscale tello_interface)

add_executable(tello_camera_interface  src/source/tello_camera_interface.cpp)
add_dependencies(tello_camera_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_camera_interface tello_interface_process)
target_link_libraries(tello_camera_interface ${catkin_LIBRARIES})

add_executable(tello_command_interface  src/source/tello_command_interface.cpp)
add_dependencies(tello_command_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_command_interface tello_interface_process)
target_link_libraries(tello_command_interface ${catkin_LIBRARIES})

add_executable(tello_state_interface  src/source/tello_state_interface.cpp)
add_dependencies(tello_state_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_state_interface tello_interface_process)
target_link_libraries(tello_state_interface ${catkin_LIBRARIES})

#############
## Testing ##
#############
#if (CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(tello_interface_test ${DRIVER_TELLO_TEST_DIR}/tello_interface_test.cpp)
#  target_link_libraries(tello_interface_test tello_interface_process)
#  target_link_libraries(tello_interface_test ${catkin_LIBRARIES})
#endif()

set_target_properties(tello_interface_process PROPERTIES LINKER_LANGUAGE CXX)