cmake_minimum_required(VERSION 3.0.2)
#SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
# SET(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
project(mqtt_comm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  message_generation
  dynamic_reconfigure
  tf
  rospy
  common
  gps
  data_comm
)

find_package(Threads REQUIRED)

# add_message_files(FILES path_point.msg task.msg agvstate.msg dispatch_hb.msg)

add_message_files(FILES
path_point_action.msg 
path_point.msg

mqtt_task.msg
mqtt_controls.msg

resp_agvstate.msg
resp_task.msg
resp_video.msg
resp_ctrl.msg

task.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES sensors
  CATKIN_DEPENDS roscpp std_msgs message_runtime common
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(mqtt_comm src/mqtt_comm.cpp)
add_dependencies(mqtt_comm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mqtt_comm ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS mqtt_comm RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/run_test.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(FILES config/params.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)





