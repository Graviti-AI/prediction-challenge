# Predictor #

prediction-challenge predictor application

## Description

The client will use `on_env` to store the historical trajectory (10 frames) sent by the simulator, and use `fetch_my_state` to get the predicted results (30 frames).

In the `./predictor/*`,

- `predictor.py` is an abstract class, you need to implement all the abstract methods in your own predictor.
- `echo_predictor.py` is a simple echo predictor, namely returns back the last frame in the historical trajectory.
- `lstm_predictor.py` is a pytorch-based predictor which is trained on the `interaction` dataset and has 0.3 MoN performance  on the MA scenario. `lstm.pt` stores the model parameters.

## Prerequisites ##

 - python 3.7

 - docker

 - grpc && protobuf

    - please follow the instruction in the `simulator/readme.md`

    - ```bash
      pip install grpcio
      pip install protobuf
      ```

 - pytorch or tensorflow to support your predictor.

## Build and Run ##
**Prepare**

use `protoc` to generate python version protocols for communication.
each time the `simulator.proto` updated, you need generate them again.
```bash
protoc -I ../proto/ --grpc_out=. --plugin=protoc-gen-grpc=`which grpc_python_plugin` ../proto/simulator.proto
protoc -I ../proto/ --python_out=. ../proto/simulator.proto
```

If this fails, you can try
```bash
python -m grpc_tools.protoc --proto_path=../proto/ --python_out=. --grpc_python_out=. ../proto/simulator.proto
```
You may get a warning when using `grpc_tools.protoc`, but it should still execute succesfully and generate the `simulator_pb2.py` and `simulator_pb2_grpc` files (provided that you have `grpc_tools` installed).

**Run locally**

```bash
# simulator should be launched before predictor, where
# -s sepcify the simulator service address
# -p specify the simulator service port
python3 main.py -s 127.0.0.1 -p 50051
```

**Docker**

- You need to put the dependencies (pytorch, numpy, ....) in the `requirement.text`, then

```bash
cd deploy
./make_image.sh my_predictor
```

