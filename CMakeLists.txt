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
    actionlib_msgs
    message_generation
    std_msgs
    geometry_msgs
    subt_ign
    roscpp
    rospy
    sensor_msgs
    laser_geometry
    roslaunch
)

roslaunch_add_file_check(launch)

### find_package(gazebo REQUIRED)



########################
## Message generation ##
########################

# Add here SubT ROS messages.

add_action_files(
  DIRECTORY action 
  FILES go_to_goal.action 
)

add_message_files(
  FILES 
  Turn.msg
)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy 
  std_msgs
  actionlib_msgs 
  geometry_msgs
)




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

add_dependencies(base_control_node ${cpp_python_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".git" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".git" EXCLUDE)

# Mark executables and/or libraries for installation
install(TARGETS base_control_node laserscan_to_pointcloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)