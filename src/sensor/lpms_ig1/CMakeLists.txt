cmake_minimum_required(VERSION 2.8.3)
project(lpms_ig1)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  common
)


generate_messages(
   DEPENDENCIES
   std_msgs
)

link_directories("${IG1_LIB}")

set(lpms_ig1_node_SRCS
    src/lpms_ig1_node.cpp
)

set(lpms_ig1_rs485_node_SRCS
    src/lpms_ig1_rs485_node.cpp
)

set(lpms_ig1_rs485_client_SRCS
    src/lpms_ig1_rs485_client.cpp
)

set(lpms_be1_node_SRCS
    src/lpms_be1_node.cpp
)

set(lpms_nav3_node_SRCS
    src/lpms_nav3_node.cpp
)

set(lpms_curs3_node_SRCS
    src/lpms_curs3_node.cpp
)

## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS})

# lpms_ig1_node
add_executable(lpms_ig1_node ${lpms_ig1_node_SRCS})
target_link_libraries(lpms_ig1_node
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )
add_dependencies(lpms_ig1_node ${catkin_EXPORTED_TARGETS})

# lpms_ig1_rs485_node
add_executable(lpms_ig1_rs485_node ${lpms_ig1_rs485_node_SRCS})
target_link_libraries(lpms_ig1_rs485_node
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )
add_dependencies(lpms_ig1_rs485_node ${catkin_EXPORTED_TARGETS})

# lpms_ig1_rs485_client
add_executable(lpms_ig1_rs485_client ${lpms_ig1_rs485_client_SRCS})
target_link_libraries(lpms_ig1_rs485_client
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )
add_dependencies(lpms_ig1_rs485_client ${catkin_EXPORTED_TARGETS})

# lpms_be1_node
add_executable(lpms_be1_node ${lpms_be1_node_SRCS})
target_link_libraries(lpms_be1_node
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )
add_dependencies(lpms_be1_node ${catkin_EXPORTED_TARGETS})


# lpms_NAV3_node
add_executable(lpms_nav3_node ${lpms_nav3_node_SRCS})
target_link_libraries(lpms_nav3_node
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )
add_dependencies(lpms_nav3_node ${catkin_EXPORTED_TARGETS})

# lpms_CURS3_node
add_executable(lpms_curs3_node ${lpms_curs3_node_SRCS})
target_link_libraries(lpms_curs3_node
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )
add_dependencies(lpms_curs3_node ${catkin_EXPORTED_TARGETS})

# imudata_rad_to_deg_node
add_executable(imudata_rad_to_deg_node src/imudata_rad_to_deg_node.cpp)
target_link_libraries(imudata_rad_to_deg_node ${catkin_LIBRARIES})
add_dependencies(imudata_rad_to_deg_node ${catkin_EXPORTED_TARGETS})
