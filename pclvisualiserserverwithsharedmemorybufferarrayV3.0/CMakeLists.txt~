cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(server)

find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} .)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
file(GLOB SRC "*.h" "*.cpp")

add_executable (server ${SRC})
target_link_libraries (server ${PCL_LIBRARIES})
