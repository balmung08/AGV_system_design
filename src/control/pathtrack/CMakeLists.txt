cmake_minimum_required(VERSION 3.0.2)
project(pathtrack)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  # serial
  # message_generation
  # dynamic_reconfigure
  tf
  # rospy
  data_comm
  common
  mqtt_comm
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   track_state.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(pathtrack src/pathtrack.cpp)
add_dependencies(pathtrack ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pathtrack ${catkin_LIBRARIES})

add_executable(speedlimit src/speedlimit.cpp)
add_dependencies(speedlimit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speedlimit ${catkin_LIBRARIES})

add_executable(selfturnctr src/selfturnctr.cpp)
add_dependencies(selfturnctr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(selfturnctr ${catkin_LIBRARIES})

# add_executable(pathtrack_simple src/pathtrack_simple.cpp)
# add_dependencies(pathtrack_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pathtrack_simple ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

install(TARGETS pathtrack RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/run.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

