# READ ME - TODO

- EB planner doesn't work now
  - Jerky
  - Collision
    - put all the cars into the `obstacle_info`. Now it only supports one car.
  - multi-thread doesn't work
  - IRL files
  - external prediction for EB
    - rewrite `getlikelihoold`, get trajectories from outside (with equal probability)
- Now in `config`, `A star` and `Reactivate` planner means the same thing. Need to use different parameters in the `A* planner`.
- Some codes are obsoleted.  (they are not used in this version of the simulator).
  - `FourWheelModel`
  - `SimplePIDController`
  - `ReadlCarModel`
  - ....
- `constant velocity predictor` doesn't work now, because now cars follow the references, not the lanelet sequence.
- `A* planner` potential bugs
  - Now the planner would use future `40 m`  to process obstacles. We found `30m` and `50m` would cause some problems.
  - Now the planner would use the condition `s_b - s_f > 8.5`  to differentiate `follow` and `obstacle`. Try other methods.
  - Now the speed profile of the planner still has some big-jerk points, try to mitigate them.
  - The new optimization about the constraints on jerk doesn't work.
  - Now the planner considers only one of the prediction trajectories.
- `Aobehavior` potential bugs
  - `40m`.
  - Now it considers only one of the prediction trajectories when processing obstacles.
- Now the simulator doesn't send `lane_id, s, d` to the client, because the python predictor won't use those information.
  - You can add them in `core::Trajectory Simulator::ToTraj(Agent* agent)`, `core/my_impl/Simulator/Simulator.cpp`. There is a `TODO` tag.

