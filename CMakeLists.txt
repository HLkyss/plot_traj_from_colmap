cmake_minimum_required(VERSION 2.8)
project(track)

#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")

find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
add_executable(test test.cpp)
target_link_libraries(demo ${Pangolin_LIBRARIES})
