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
      # Install grpc library (if not installed)
      pip install grpcio			# my grpcio version is 1.30.0
      ```

 - pytorch or tensorflow to support your predictor.

## Build and Run ##
I recommend that you can run the predictor locally, but run the simulator in the docker.

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

