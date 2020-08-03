# CMake instructions to make the static lib
cmake_minimum_required(VERSION 3.12)
project(PidControllerLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(./include/PidControllerLib)

ADD_LIBRARY( PidControllerLib STATIC
             src/PidController.cpp
)

target_link_libraries(PidControllerLib m)
