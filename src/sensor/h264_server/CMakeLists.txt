cmake_minimum_required(VERSION 3.10)
project(h264_server)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)


find_package(OpenCV REQUIRED)

find_package(PkgConfig)
# pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  roscpp
  std_msgs
  cv_bridge
  audio_common_msgs
)


link_libraries("/usr/lib/x86_64-linux-gnu/libva.so.2")
link_libraries("/usr/lib/x86_64-linux-gnu/libswscale.so.5")
# link_libraries("/usr/lib/x86_64-linux-gnu/libv4l/v4l1compat.so")

include_directories(
  include
  /usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${GST1.0_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib -lavformat -lavcodec -lavutil
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp opencv std_msgs cv_bridge
)

# link_directories(
#   /usr/local/lib
# )

add_executable(${PROJECT_NAME}_node src/codec_server.cpp src/push_h264.cpp)

target_include_directories(${PROJECT_NAME}_node  PRIVATE ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} pthread  avcodec avutil avformat)

# add_executable(push_acc src/push_acc.cpp)
# target_include_directories(push_acc  PRIVATE ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR})
# target_link_libraries(push_acc PRIVATE ${catkin_LIBRARIES} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${GST1.0_LIBRARIES} pthread  avcodec avutil avformat)

add_executable(rtmp_stream src/rtmp_stream.cpp)
target_link_libraries(rtmp_stream PRIVATE ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS h264_server_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/run.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )