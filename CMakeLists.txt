cmake_minimum_required(VERSION 2.8)
project(track)

#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})

add_executable(test test.cpp)
add_executable(test2 test2.cpp)
add_executable(demo demo.cpp)
target_link_libraries(demo ${Pangolin_LIBRARIES})