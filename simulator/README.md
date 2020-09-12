# Simulator #

prediction-challenge simulator application

## Description

This simulator will first generate a car with `car_id = 0`, then wait for the client (a python predictor) to connect.

The client can use `fetchEnv` to get the past 10 frames (1s) trajectory of the car 0, and use `onUserState` to upload the predicted results (30 frames, 3s) to the simulator. 

## Prerequisites ##

 - docker
    - follow the official documentations
 - gcc & g++ with c++14 support
 - cmake >= v3.13

```bash
# Install cmake (if not installed)

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

- gRPC

```bash
# Install grpc (if not installed)

git clone -b v1.28.2 https://github.com/grpc/grpc.git
cd grpc && git submodule update --init
mkdir build/cpp && cd build/cpp
cmake -DCMAKE_BUILD_TYPE=Release ../..
make -j4
make install
```

 - Lanelet2
    - make sure you have all the dependencies of Lanelet2. It can be find here: https://github.com/fzi-forschungszentrum-informatik/Lanelet2. (You don't need download Lanelet2. It is already included in the folder.)

## Build and Run ##

**To build and run the simulator locally:**

1. copy `./libsim.a` to `/usr/lib/`
2. copy `./libjsoncpp.a` to `/usr/lib/`
3. use `protoc` to generate C++ version protocols for communication

```bash
protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
protoc -I ../proto/ --cpp_out=./service/proto/ ../proto/simulator.proto
```

3. cmake

 ```bash
mkdir build && cd build
cmake ..
make -j4
./simulator -p 50051

# If you want to load config
./simulator -p 50051 -c ../config.txt

# If you want to rviz visualize, you need to open port 8086 for rviz
./simulator -p 50051 -r 8086
 ```

**Using Docker**

- Remember to delete `./build` and  `./service/proto/*` (which are the relics after building locally).
- Run in the image. (the image will use `port 50051` and `8086`)

 ```bash
 cd depoly
 ./build_and_run_in_container.sh		# maybe you need sudo
 ```

**Log**: log files will be stored in `./Log`. The order of numbers is  `id, x, y, yaw, vx, vy, vyaw, length, width, lane_id`.

**Config**: see `./config.txt`.

## Logs

**9/5/20 SYF**

- new `.proto`; support car number > 1;
- support log;
- modify `BehaveCar` in simulator.

- start from `config.txt`; **see `./core/my_impl/my_simulator_impl.cpp` line 31 - line 35, `void MySimulatorImpl::start() `**. To choose start from `config.txt` or randomly sampling.

**8/20/20 SYF**

- Modify `fetchEnv` and `onUserState`
- add rviz visualization

**8/12/20 SYF**

- I modify the `onUserState` function:
  - See `./core/my_impl/my_simulator_impl.cpp` line 50. After printing the the `traj` from the client, this function will call `simulator.upload_traj(0, traj)` to upload the `traj`.
  - See `./core/my_impl/Simulator/Simulator.cpp` line 315. This function will put the `traj` in the buffer into `agent->getPredictor()`.
  - See `./core/mu_impl/Predictors/PyPredictor.cpp`. This class will store the `traj` from `Client` into a buffer, then return it in `Update()`.

**8/7/20 SYF**

- The simulator will first generate a car with `car_id = 0`, and then waiting for  `onUserState`  and `fetchEnv`.
  - `fetchEnv`:
    - see `./core/my_impl/Simulator/Simulator.cpp` line 32(`#define Car_Num 1 `), line 62 (`int id = 0;`), line 187 (`generateBehaveCar()`), line 209-212. The simulator will generate a car  with `car_id = 0` at beginning.
    - see line 261 `core::Trajectory Simulator::randomly_sample(int car_id)`. line 263 - 305 are copied from `Simulator::run()`, which will simulate the situation at the next `Tick`. 
    - see line 308-353. The simulator will search a car satisfying the input `car_id`, and return the last state to client. If not find, this function will return a default value (`233`).
  - `onUserState`:
    - it will simply print the `traj` from the client.

**8/4/20 SYF**

I modify the old simulator program to fit the gRPC framework. The simulator-related codes are in the `./core/my_impl`. Maybe it will be kind of formidable for the first glance. Here are some instructions:

- There are various types of different `class`  here. I suggest you can check the documentation in `./core/my_impl/Docs` for help.
- The core part of the simulator is in `./core/my_impl/Simulator/Simulator.cpp`. By now, when `MySimulatorImpl::start` triggers, the simulator will first generate cars and then `updatetick` in each time step.
- See `./service/service.cpp` line 36. Now the simulator will automatically run and won't listen for clients. You can omit that line, and use `command` from the client to control the simulator. Maybe it needs to change the `simulator.proto` to let the `message` contain the `command`.
- See `./core/my_impl/Agents/Agent.hpp` line 71, the historical information is stored in `std::vector<Vector>  preState` for each car. See `./core/my_impl/Simulator/Simulator.hpp` line 74 - line 85, you can get the car id by these variables. So You can let client to specify which car ID you want to ask historical information, and use  `preState` to return it. 
- Now the simulator doesn't use any `track_info.csv`, and just randomly generate the origin location for each car, and map it to the road.
