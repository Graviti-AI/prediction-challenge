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

**To build and run the simulator locally:**
 ```bash
 protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
 protoc -I ../proto/ --cpp_out=./service/proto/ ../proto/simulator.proto

 mkdir build && cd build
 cmake ..
 make
 ./simulator
 ```

**To build and run the simulator with docker:**
 ```bash
 cd depoly
 ./build_and_run_in_container.sh
 ```
