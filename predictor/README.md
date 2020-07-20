# predictor #

prediction-challenge predictor application

## Prerequisites ##

 - python3.7
 - docker
 - grpc && protobuf

## Build and Run ##
**Install grpc library (if not installed)**
```bash
pip install grpcio
```

**Create predictor**
> **TODO**: add description for predictor.predictor.Predictor
>
**Run Predictor**
```bash
# simulator should be launched before predictor, where
# -s sepcify the simulator service address
# -p specify the simulator service port
python3 main.py -s 127.0.0.1 -p 50051
```
