cmake_minimum_required(VERSION 3.0.2)
project(dwsubt)


if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    subt_ign
    roscpp
    sensor_msgs
    laser_geometry
)

### find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories( include 
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(base_control_node src/base_control_node.cc src/mission.cpp)
add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)

add_dependencies(base_control_node ${catkin_EXPORTED_TARGETS})
add_dependencies(laserscan_to_pointcloud ${catkin_EXPORTED_TARGETS})

target_link_libraries(base_control_node ${catkin_LIBRARIES})
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})



###########
## Tests ##
###########


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Mark executables and/or libraries for installation
install(TARGETS base_control_node laserscan_to_pointcloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)