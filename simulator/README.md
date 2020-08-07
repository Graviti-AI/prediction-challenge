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

### Updates

**8/4/20 SYF**

I modify the old simulator program to fit the gRPC framework. The simulator-related codes are in the `./core/my_impl`. Maybe it will be kind of formidable for the first glance. Here are some instructions:

- There are various types of different `class`  here. I suggest you can check the documentation in `./core/my_impl/Docs` for help.
- The core part of the simulator is in `./core/my_impl/Simulator/Simulator.cpp`. By now, when `MySimulatorImpl::start` triggers, the simulator will first generate cars and then `updatetick` in each time step.
- See `./service/service.cpp` line 36. Now the simulator will automatically run and won't listen for clients. You can omit that line, and use `command` from the client to control the simulator. Maybe it needs to change the `simulator.proto` to let the `message` contain the `command`.
- See `./core/my_impl/Agents/Agent.hpp` line 71, the historical information is stored in `std::vector<Vector>  preState` for each car. See `./core/my_impl/Simulator/Simulator.hpp` line 74 - line 85, you can get the car id by these variables. So You can let client to specify which car ID you want to ask historical information, and use  `preState` to return it. 
- Now the simulator doesn't use any `track_info.csv`, and just randomly generate the origin location for each car, and map it to the road.

**8/7/20 SYF**

- The simulator will first generate a car with `car_id = 0`, and then waiting for  `onUserState`  and `fetchEnv`.
  - `fetchEnv`:
    - see `/core/my_impl/Simulator/Simulator.cpp` line 32(`#define Car_Num 1 `), line 62 (`int id = 0;`), line 187 (`generateBehaveCar()`), line 209-212. The simulator will generate a car  with `car_id = 0` at beginning.
    - see line 261 `core::Trajectory Simulator::randomly_sample(int car_id)`. line 263 - 305 are copied from `Simulator::run()`, which will simulate the situation at the next `Tick`. 
    - see line 308-353. The simulator will search a car satisfying the input `car_id`, and return the last state to client. If not find, this function will return a default value (`233`).
  -  `onUserState`:
    - it will simply print the `traj` from the client.