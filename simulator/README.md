# README - Simulator #

This folder contains the simulator code for the `INTERPRET Challenge`. 

### Description

This simulator needs to be started with the `config` specified (see the config format in `conf/ConfigRM.md`). The `config` file would list three types of cars: 

- `robot car` (controlled by internal planning algorithms).
- `replay car with prediction` (tracking trajectory in the dataset, and `robot car`  can 'see' their current states and prediction when making decisions).
- `replay car without prediction` (tracking trajectory in the dataset, and `robot car` cannot see them, but `replay car with prediction` would consider them into the prediction).

The features of this simulator are:

- Support some internal planning algorithms, like IDM, A*, and EB.
- Replay dataset and simultaneously simulate `robot car`.
- Support external prediction algorithms in python. The communication part is based on `gRPC`. We provide some examples in `../predictors`.

The pipeline of our simulator is:

```
1. Load the config file. (see core/my_impl/Simulator/Simulator.cpp, void InitSimulation())

2. Let `agents` = [], `replay_agents` = []. (see core/my_impl/Simulator/Simulator.cpp, void run())
3. while True:
    3.1 keep sleep() until all the agents from `agents` and `replay_agents` have finished last_step's updates.
    3.2 set the states of all the agents as `ready`, then start the next_step's updates.
    3.3 remove agents from `agents` and `replay_agents` if they have reached the terminates.
    3.4 add agents to `agents` and `replay_agents` if the config file specifies some cars to be added at this timestep.
    3.5 if current_update_times == MaxUpdateTimes, then break.
    3.6 use multi-thread to update each agent from `agents`. (see core/my_impl/Agents/BehaveCar.cpp, void Run())
    	3.6.1 get planning results.
    	3.6.2 update prediction results (if the predictor is from the external python side, it will wait until receiving the results).
    3.7 use multi-thread to update each agent from `replay_agents`. (see core/my_impl/Behaviours/ReplayGenerator.cpp, void Run())
    	3.7.1 get next states from dataset.
    	3.7.2 update prediction results (if the predictor is from the external python side .......)
    3.8 GOTO 3.1
```

In general, this simulator has two main threads, one is used to run the above pipeline, another is used to communicate with the client based on gRPC. Once there is any car waiting for calculating the prediction, the first thread would wait, meanwhile the second would  give the input to the client and set the predicted results back to that car, then the first thread would go on.

If the config file allows the external python predictor, our simulator would open the port `50051`, and use `gRPC` to listen the requests from the python side. Actually,  our simulator acts as the `server`, and the predictor acts as the `client`. The `client` would repeatedly ask the `servers` whether there is any car waiting for the external prediction results, if any, the `client` would calculate for the `server`. Remember that after you run the `simulator`, you also have to run the `predictor` (see guides in `../predictors`.)

- About how to fetch the input for the `client`, see `core/my_impl/Simulator/Simualtor.cpp, core::SimulationEnv Simulator::fetch_history()`.
- About how to receive the prediction results  from the `client`, see `core/my_impl/Simulator/Simualtor.cpp, void Simulator::upload_traj()`.
- The predictor needs the past 10 frames (1s) trajectories of all the cars as input and predicts the future 30 frames (3s) trajectory for the ego car.

For more detailed APIs, please refer to `README_API.docx`.

For TODO-LIST, please refer to `README_TODO.md`.

### Requirements

- Docker (please follow the official documentations).
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

 - The **dependencies** of Lanelet2 (please refer to https://github.com/fzi-forschungszentrum-informatik/Lanelet2). **NOTE** that you do not have to download Lanelet2, since it is already included in the folder (see `core/my_impl/Lanelet2Lib`).

### Build and Run

**Build and run locally:**

- Use `protoc` to generate C++ version `gRPC` protocols.

```bash
mkdir ./service/proto/

protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
protoc -I ../proto/ --cpp_out=./service/proto/ ../proto/simulator.proto
```

- Use `cmake`

 ```bash
mkdir build && cd build
cmake ..
make -j4
./simulator -c ../conf/test_config_0.txt -l ../Log/config$i [-p port] [-r rviz_port] [--verbose]

# default port is 50051
 ```

**Build and run in the docker**

- Delete `./build` and  `./service/proto/*`  (they would conflict with files in the docker), but keep `./service/proto/`.

 ```bash
 cd deploy
 ./build_and_run_in_container.sh		# maybe you need sudo
 ```

**Recompile `libsim.a`**

- If you have the access to `core/my_impl/planner/*.cpp` and `core/my_impl/plannerLib/*.cpp`, once you change those `.cpp`, you need to recompile `libsim.a`.

```bash
mkdir build && cd build
cmake -DLIBSIM=ON ..
make -j4
cp libsim.a ../
```
