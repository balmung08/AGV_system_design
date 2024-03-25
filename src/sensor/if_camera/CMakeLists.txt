cmake_minimum_required(VERSION 3.0.2)
project(if_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS  Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/IRNetDemo/main.cpp
  src/IRNetDemo/mainwindow.cpp
  src/IRNetDemo/irnetsdkmanager.cpp
  src/IRNetDemo/playerwidget.cpp
  src/IRNetDemo/utils/config.cpp
  src/IRNetDemo/analysisgraphicsitem.cpp
  src/IRNetDemo/analysisgraphicswidget.cpp
)
set(FORMS
  src/IRNetDemo/mainwindow.ui
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES if_camera
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#link_directories(${catkin_LIB_DIRS} lib)

link_directories(
  lib
  # -lIRNet -lrtspclient
)



add_executable(if_cam ${SOURCES} ${FORMS} )
target_link_libraries(if_cam  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Widgets IRNet rtspclient)

add_executable(read_if_cam src/main.cpp 
  src/IRNetDemo/irnetsdkmanager.cpp
  src/IRNetDemo/playerwidget.cpp
  src/IRNetDemo/utils/config.cpp
  src/IRNetDemo/analysisgraphicsitem.cpp
  src/IRNetDemo/analysisgraphicswidget.cpp
)
target_link_libraries(read_if_cam  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Widgets IRNet rtspclient)

