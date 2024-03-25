cmake_minimum_required(VERSION 3.0.2)
project(turntable)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  common
)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(turntable src/turntable.cpp)
add_dependencies(turntable ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turntable ${catkin_LIBRARIES})

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
   src/panel/turntable_panel.h
)

set(GUI_SOURCE_FILES
  src/panel/turntable_panel.cpp 
  ${MOC_FILES})
add_library(turntable_gui ${GUI_SOURCE_FILES})
target_link_libraries(turntable_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  turntable_gui
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
install(TARGETS turntable RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES launch/run.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )  
