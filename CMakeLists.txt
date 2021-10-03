cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  livox_ros_driver
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## Generate messages in the 'msg' folder
add_message_files(FILES
	CustomPoint.msg
	CustomMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
)

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-7/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_simulation
        CATKIN_DEPENDS tf
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation
                ${catkin_LIBRARIES} RayPlugin
                libprotobuf.so.9)


### TEST ###
add_executable(point_order_test src/test/point_order_test.cpp)
target_link_libraries(point_order_test
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_executable(point_line_test src/test/point_line_test.cpp)
target_link_libraries(point_line_test
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

### INSTALL ###
install(TARGETS livox_laser_simulation
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
