## Implementing a Python Planner 

**Background**: The self-driving car Simulator process communicates with a Planner process via interprocess communication (IPC) through gRPC. Having the Planner in a separate process from the Simulator allows for parallelism when working in a simulation with many thousands of cars. For simplicity, we have implemented the Planner abstract base class which can be found in predictor/planner/planner.py. The `__main__` function (called in simulator_client.py) manages the communication of the gRPC stubs and marshalling, which is abstracted away from the user of the Python base class.

**Objective**: This document describes how to implement the Planner wrapper class so that researchers can experiment with their own planning algorithms in Python. It also provides background on the back-end C++ implementation of PyPlanner for developers wishing to extend the functionality of Python Planners. 

**Implementation:**
Users wishing to implement their own Planning Algorithms should extend the Planner.py class, which implements the following interface: 

```
Planner.__init__()
Planner.start()
Planner.shutdown()
Planner.on_env(map_name, my_traj, other_trajs)
Planner.fetch_my_plan()
```

`__init__(self)`: Initializer. Create any member fields here (for example, you may want to hold onto the trajectory). Note that this is shared by all threads. 
	Arguments: Implementation-dependent (may be none)

`start(self)`:  Perform any thread-specific setup here. This is separate from the initializer because you may want to have different behavior for different threads within the same process.
Arguments: None
Returns: None

`on_env(self, map_name, reference_points，my_traj, other_trajs, obstacle_info)`: This function receives the car’s trajectory and the trajectories of nearby cars from the simulator process. This is where you should do most of the work and generate the plan. It may be desirable to fork a subprocess/thread to do the work of generating the plan and return quickly to the caller. Note that any plan generated here would need to be saved as an attribute of the class, as it is only retrieved when the fetch_my_plan() function below is called.  
Arguments:  
`map_name`: str representing the name of the map that is being simulated   
`reference_points`: a list of `States` that are the reference for planning (may not be used)  
`my_traj`: The `Trajectory` of the vehicle so far. Trajectory is simply a wrapper class for a list of the States (see details below)  
`other_trajs`: List of `Trajectory`’s of the nearby vehicles (may not be used by planner)  
`obstacle_info`: List of `Obstacle`’s. Each obstacle is identified by a list of points in the obstacle and the distance from the agent to the obstacle (see details below)  
Returns: None (Any returns from this function are ignored)  

`fetch_my_plan(self)`: This function should return a Trajectory describing the vehicle plan. The output of the planning algorithm should be returned here so that the simulator process can take it into account. If the on_env() call returned quickly without generating the full plan, you may have to block here until the plan is ready before returning it.  
Arguments: None  
Returns: `Trajectory` object (see details below) which describes the planned trajectory of the vehicle  

`shutdown(self)`: Perform any thread-specific teardown here. Cleanup from start(self)  

Arguments: None  
Returns: None  

Objects used (code in predictor/traj.py)
State Object fields:  
```
self.track_id; 
self.frame_id;
self.timestamp_ms;
self.agent_type;
self.x; 	# x-position
self.y; 	# y-position
self.vx;	# x velocity
self.vy;	# y velocity
self.psi_rad;
self.length;
self.width;
self.jerk;
self.current_lanelet_id;
self.s_of_current_lanelet;
self.d_of_current_lanelet;
```

Trajectory Object holds simply a list of states. Use 
```
self.get_states() # to retrieve the list of states in order.
self.append_state(state) # to put a new state at the end of the list.
```

Obstacle Object includes information about a single obstacle. 
```
self.point_in; # list of State Objects (points) that are in the obstacle
self.distance; # distance from the agent to obstacle
self.yielding; # default True
```
	    
Note that the `simulator_client` python process manages two different sets of tasks: Planning algorithms, and Prediction ML models. Since the planners and the predictors would never actually be executing concurrently (ie. all planners run in parallel first, then we transfer control back to the simulator server, then all the predictors run in parallel), we will have them both being run in the same process, whose entrypoint is `simulator_client.py`. 
The `simulator_client.py` manages the communication with gRPC stubs. It interfaces with both the `Predictor.py` and `Planner.py` classes.
