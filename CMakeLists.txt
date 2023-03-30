cmake_minimum_required(VERSION 3.10.2)
project(livox_laser_simulation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf
  livox_ros_driver
)

## Find gazebo
if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_simulation
        CATKIN_DEPENDS tf
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(TARGETS livox_laser_simulation
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY
        launch
        meshes
        resources
        rviz
        scan_mode
        urdf
        worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
