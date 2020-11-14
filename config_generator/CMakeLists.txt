cmake_minimum_required(VERSION 3.10)
project(simu)
SET(CMAKE_CXX_FLAGS -pthread)

add_subdirectory(Lanelet2Lib)
include_directories(
	${LANELET2_INCLS_DIRS}
    )


set(CMAKE_CXX_STANDARD 14)
file(GLOB SRC_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*
        ${CMAKE_CURRENT_SOURCE_DIR}/alglib/*
        )
add_executable(simu ${SRC_FILES} main.cpp)
target_link_libraries (simu lanelet2) 