# prediction-challenge

This repo includes the code for `INTERPERT Challenge`.

- `proto`: The definition of `message` communicated between the server and the client.
- `config_generator`: The generator of configuration files .
- `simulator`: The server (C++) side.
- `predictors`: The client (python) side.
- `rviz_visulization`: The real-time visualization tool based on `ROS`.
- `py_replay`: The visualization toolkit.
- `metrics`: The metrics calculator.

### NOTE

1. If you want to modify the communication content, you can go through `proto` and add/remove some terms. You also need to modify the `gRPC` part in the `simulator` and `predictor`.
2. If you want to test the `predictor`, first, you need to run the `simulator` (see the command in that folder) and then run the `predictor` (see the command in that folder). After you finish it, you can copy the log files from the `simulator/Log` to `py_replay/Log` and use visualization tools to debug or measure the performance.
3. The `predictors` folder includes some `predictor`s, You can try any of them.

### Test the Simulator

Here is the summary of all the command lines to run this simulator. You can simply run them in order without reading code.

For installing dependencies, please see readme(s) in the corresponding folders.

```bash
git clone xxx  # this repo link
cd prediction-challenge/  # this repo folder

# Terminate 1: run the simulator
cd simulator/  # then follow the ./simulator/README.md to install gRPC, cmake ...
mkdir Log/

# generate proto
mkdir ./service/proto/
protoc -I ../proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ../proto/simulator.proto
protoc -I ../proto/ --cpp_out=./service/proto/ ../proto/simulator.proto

mkdir build && cd build
cmake ..
make -j4
./simulator -c ../conf/test_config_0.txt -l ../Log/config0


# Now the simulator is running, open another terminate to run the predictor.
cd prediction-challenge/
cd predictors/	

# generate proto
protoc -I ../proto/ --grpc_out=. --plugin=protoc-gen-grpc=`which grpc_python_plugin` ../proto/simulator.proto
protoc -I ../proto/ --python_out=. ../proto/simulator.proto
mv simulator_pb2.py predictor_lstm/ 	# we provide some predictors, you can try any of them
mv simulator_pb2_grpc.py predictor_lstm/ 
cd predictor_lstm/ 	

# follow the readme and requirement.txt to install gRPC, pytorch, ...
python main.py


# Now, the simulator and the predictor should run normally
# You can follow the readme in py_replay/ to visualize the logs.
```

