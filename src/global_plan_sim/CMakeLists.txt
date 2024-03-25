cmake_minimum_required(VERSION 3.0.2)
project(global_plan_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  tf
  common
  gps
  data_comm
  mqtt_comm
)

find_package(Threads REQUIRED)

# add_message_files(FILES path_point.msg task.msg agv_state.msg)

# generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES sensors
  CATKIN_DEPENDS roscpp std_msgs message_runtime common
#  DEPENDS system_lib
)

# catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(local_path_plan src/localpathplan.cpp)
add_dependencies(local_path_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_path_plan ${catkin_LIBRARIES})

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
   src/panel/global_plan_sim_panel.h
   src/panel/node_check.h
)

set(GUI_SOURCE_FILES
  src/panel/global_plan_sim_panel.cpp 
  src/panel/node_check.cpp 
  ${MOC_FILES}
)
add_library(global_plan_sim_gui ${GUI_SOURCE_FILES})
target_link_libraries(global_plan_sim_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  global_plan_sim_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Install ##
#############

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS local_path_plan RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/run_all_new.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )  
install(FILES show.rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )  
install(FILES path/config1.yaml path/config2.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/path )  
