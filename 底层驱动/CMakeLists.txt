cmake_minimum_required(VERSION 3.14)

project(Car_8drv_Lite)

set(CMAKE_CXX_STANDARD 14)

aux_source_directory(. DIRSRCS)

add_executable(${PROJECT_NAME} ${DIRSRCS})

find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} Threads::Threads)

