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

### Updates

**8/7/20 SYF**

- `./predictor/echo_predictor` is the implement of the echo predictor. It will store the last fetched trajectory from the simulator (see `on_env`), then return it back to the simulator (see `fetch_my_state`).
- **If you want to build your own predictor, you can refer `predictor/predictor.py` and `predictor/echo_predictor.py`**
- 