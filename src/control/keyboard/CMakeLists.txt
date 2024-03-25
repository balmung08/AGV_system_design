cmake_minimum_required(VERSION 3.0.2)
project(keyboard)


find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )



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



add_executable(keyboard
  src/keyboard.cpp
)
add_dependencies(keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS keyboard RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES launch/run.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

