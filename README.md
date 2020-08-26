# prediction-challenge

For simulator part, see
* [simulator](https://github.com/Graviti-AI/prediction-challenge/tree/master/simulator)

For predictor part, see
* [predictor](https://github.com/Graviti-AI/prediction-challenge/tree/master/predictor)


**8/26/20 SYF**

To meet the demand of the simulator and the preditors, the `simulator.proto` has been changed (see `proto/simulator_new.proto`). Please help me change the corresponding code to fit the new `.proto`. For example, these following files need to be modified.

- `simulator/service/*`, `simulator/core/trajectory.h`, `simulator/core/imp/default_simulator_impl.h`, etc.
- `predictor/simulator_client.py`, `predictor/predictor/*`, etc.
