cmake_minimum_required(VERSION 3.0.2)
project(common)


find_package(catkin REQUIRED
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES public can 
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(public src/public.cpp)
add_dependencies(public ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(public ${catkin_LIBRARIES})

add_library(can src/can.cpp)
add_dependencies(can ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(can ${catkin_LIBRARIES})

# add_library(mySub src/mySub.cpp)
# add_dependencies(mySub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(mySub ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS public can RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


