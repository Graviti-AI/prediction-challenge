cmake_minimum_required(VERSION 3.5.1)

project(simulator CXX)

find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
    .
)

add_executable(simulator 
    core/impl/default_simulator_impl.cpp 
    core/simulator.cpp 
    main.cpp 
    service/impl/service_impl.cpp 
    service/proto/simulator.grpc.pb.cc 
    service/proto/simulator.pb.cc 
    service/service.cpp
)

target_link_libraries(simulator gRPC::grpc++)