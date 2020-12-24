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