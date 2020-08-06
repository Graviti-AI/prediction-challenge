# simulator #

prediction-challenge simulator application

## Prerequisites ##

 - cmake >= v3.13
 - docker
 - gcc & g++ with c++14 support

## Build and Run ##
**Install cmake (if not installed)**
```bash
sudo apt install -y cmake
git clone -b v3.17.3 https://github.com/Kitware/CMake.git
mkdir CMake/build
cd CMake/build
cmake ..
make -j4
make install
sudo apt remove -y cmake
ln -s /usr/local/bin/cmake /usr/bin/cmake
```

**Install grpc (if not installed)**
```bash
git clone -b v1.28.2 https://github.com/grpc/grpc.git
cd grpc && git submodule update --init
mkdir build/cpp && cd build/cpp
cmake -DCMAKE_BUILD_TYPE=Release ../..
make -j4
make install
```

**Lanelet2 and other dependencies for the simulator**

1. make sure you have all the dependencies of Lanelet2. It can be find here: https://github.com/fzi-forschungszentrum-informatik/Lanelet2. (You don't need download Lanelet2. It is already included in the folder.)

2. copy ./libsim.a to /usr/lib/


**To build and run the simulator locally:**

 ```bash
 protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
 protoc -I ../proto/ --cpp_out=./service/proto/ ../proto/simulator.proto

 mkdir build && cd build
 cmake ..
 make
 ./simulator
 ```

**Docker**

 ```bash
 cd depoly
 ./build_and_run_in_container.sh
 ```

**Error Log:**

```bash
Sending build context to Docker daemon   34.3kB
Step 1/10 : FROM hub.graviti.cn/prediction-challenge/simulator-base:1.0 AS simulator-build-env
 ---> a4a067ea2550
Step 2/10 : LABEL label-simulator-build-env=simulator-build-env
 ---> Running in 6007b26d84ac
Removing intermediate container 6007b26d84ac
 ---> 66b0ba75ce4b
Step 3/10 : RUN mkdir -p /prediction-challenge/simulator_src/build
 ---> Running in 29003d5e3f5e
Removing intermediate container 29003d5e3f5e
 ---> 3a083d39773d
Step 4/10 : COPY ./. /prediction-challenge/simulator_src/
 ---> b54080fd8b12
Step 5/10 : RUN cd /prediction-challenge/simulator_src/build && cmake .. && make
 ---> Running in d558e19fb455
-- The CXX compiler identification is GNU 7.5.0
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ - works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Looking for C++ include pthread.h
-- Looking for C++ include pthread.h - found
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD - Failed
-- Looking for pthread_create in pthreads
-- Looking for pthread_create in pthreads - not found
-- Looking for pthread_create in pthread
-- Looking for pthread_create in pthread - found
-- Found Threads: TRUE  
-- Configuring done
CMake Error at CMakeLists.txt:15 (add_executable):
  Cannot find source file:

    service/proto/simulator.grpc.pb.cc

  Tried extensions .c .C .c++ .cc .cpp .cxx .cu .m .M .mm .h .hh .h++ .hm
  .hpp .hxx .in .txx


CMake Error at CMakeLists.txt:15 (add_executable):
  No SOURCES given to target: simulator


CMake Generate step failed.  Build files cannot be regenerated correctly.
The command '/bin/sh -c cd /prediction-challenge/simulator_src/build && cmake .. && make' returned a non-zero code: 1
Total reclaimed space: 0B
./build_and_run_in_container.sh: line 13: docker-compose: command not found
```

If I generate the local `protoc` file, the error turns out to be:

```bash
Step 5/10 : RUN cd /prediction-challenge/simulator_src/build && cmake .. && make
 ---> Running in bfdfcbf2b536
-- The CXX compiler identification is GNU 7.5.0
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ - works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Looking for C++ include pthread.h
-- Looking for C++ include pthread.h - found
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD - Failed
-- Looking for pthread_create in pthreads
-- Looking for pthread_create in pthreads - not found
-- Looking for pthread_create in pthread
-- Looking for pthread_create in pthread - found
-- Found Threads: TRUE  
-- Configuring done
-- Generating done
-- Build files have been written to: /prediction-challenge/simulator_src/build
Scanning dependencies of target simulator
[ 12%] Building CXX object CMakeFiles/simulator.dir/core/impl/default_simulator_impl.cpp.o
[ 25%] Building CXX object CMakeFiles/simulator.dir/core/simulator.cpp.o
[ 37%] Building CXX object CMakeFiles/simulator.dir/main.cpp.o
[ 50%] Building CXX object CMakeFiles/simulator.dir/service/impl/service_impl.cpp.o
In file included from /prediction-challenge/simulator_src/./service/proto/simulator.grpc.pb.h:7:0,
                 from /prediction-challenge/simulator_src/service/impl/service_impl.h:2,
                 from /prediction-challenge/simulator_src/service/impl/service_impl.cpp:1:
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:17:2: error: #error This file was generated by an older version of protoc which is
 #error This file was generated by an older version of protoc which is
  ^~~~~
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:18:2: error: #error incompatible with your Protocol Buffer headers. Please
 #error incompatible with your Protocol Buffer headers. Please
  ^~~~~
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:19:2: error: #error regenerate this file with a newer version of protoc.
 #error regenerate this file with a newer version of protoc.
  ^~~~~
In file included from /prediction-challenge/simulator_src/./service/proto/simulator.grpc.pb.h:7:0,
                 from /prediction-challenge/simulator_src/service/impl/service_impl.h:2,
                 from /prediction-challenge/simulator_src/service/impl/service_impl.cpp:1:
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:317:38: error: 'InternalMetadataWithArena' in namespace 'google::protobuf::internal' does not name a type
   ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
                                      ^~~~~~~~~~~~~~~~~~~~~~~~~
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:464:38: error: 'InternalMetadataWithArena' in namespace 'google::protobuf::internal' does not name a type
   ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
                                      ^~~~~~~~~~~~~~~~~~~~~~~~~
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:580:38: error: 'InternalMetadataWithArena' in namespace 'google::protobuf::internal' does not name a type
   ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
                                      ^~~~~~~~~~~~~~~~~~~~~~~~~
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:740:38: error: 'InternalMetadataWithArena' in namespace 'google::protobuf::internal' does not name a type
   ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
                                      ^~~~~~~~~~~~~~~~~~~~~~~~~
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:876:38: error: 'InternalMetadataWithArena' in namespace 'google::protobuf::internal' does not name a type
   ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
                                      ^~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /prediction-challenge/simulator_src/./service/proto/simulator.grpc.pb.h:7:0,
                 from /prediction-challenge/simulator_src/service/impl/service_impl.h:2,
                 from /prediction-challenge/simulator_src/service/impl/service_impl.cpp:1:
/prediction-challenge/simulator_src/./service/proto/simulator.pb.h:1021:38: error: 'InternalMetadataWithArena' in namespace 'google::protobuf::internal' does not name a type
   ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
                                      ^~~~~~~~~~~~~~~~~~~~~~~~~
CMakeFiles/simulator.dir/build.make:121: recipe for target 'CMakeFiles/simulator.dir/service/impl/service_impl.cpp.o' failed
make[2]: *** [CMakeFiles/simulator.dir/service/impl/service_impl.cpp.o] Error 1
CMakeFiles/Makefile2:95: recipe for target 'CMakeFiles/simulator.dir/all' failed
make[1]: *** [CMakeFiles/simulator.dir/all] Error 2
Makefile:103: recipe for target 'all' failed
make: *** [all] Error 2
The command '/bin/sh -c cd /prediction-challenge/simulator_src/build && cmake .. && make' returned a non-zero code: 2
Total reclaimed space: 0B
./build_and_run_in_container.sh: line 13: docker-compose: command not found
```

It seems the versions of `protoc` are inconsistent.

If I add these commands into `dockerfile` (I want to regenerate `protoc` in the image).

```dockerfile
RUN cd /prediction-challenge/simulator_src/
RUN protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
RUN protoc -I ../proto/ --cpp_out=./service/proto/ ../proto/simulator.proto
```

The error will be :

```bash
Step 5/13 : RUN cd /prediction-challenge/simulator_src/
 ---> Running in f1f944575ea7
Removing intermediate container f1f944575ea7
 ---> 1c299b86ba4e
Step 6/13 : RUN protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
 ---> Running in 61130fd1abf3
../proto/: warning: directory does not exist.
Could not make proto path relative: ../proto/simulator.proto: No such file or directory
The command '/bin/sh -c protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto' returned a non-zero code: 1
Total reclaimed space: 0B
./build_and_run_in_container.sh: line 13: docker-compose: command not found
```

It seems the `../proto` folder doesn't exist.

## Instructions

**8/4/20 Updates**

I modify the old simulator program to fit the gRPC framework. The simulator-related codes are in the `./core/my_impl`. Maybe it will be kind of formidable for the first glance. Here are some instructions:

- There are various types of different `class`  here. I suggest you can check the documentation in `./core/my_impl/Docs` for help.
- The core part of the simulator is in `./core/my_impl/Simulator/Simulator.cpp`. By now, when `MySimulatorImpl::start` triggers, the simulator will first generate cars and then `updatetick` in each time step.
- See `./service/service.cpp` line 36. Now the simulator will automatically run and won't listen for clients. You can omit that line, and use `command` from the client to control the simulator.
- See `./core/my_impl/Agents/Agent.hpp` line 71, the historical information is stored in `std::vector<Vector>  preState` for each car. See `./core/my_impl/Simulator/Simulator.hpp` line 74 - line 85, you can get the car id by these variables. So You can let client to specify which car ID you want to ask historical information, and use  `preState` to return it. 
- Now the simulator doesn't use any `track_info.csv`, and just randomly generate the origin location for each car, and map it to the road.