# READ ME - API

[TOC]

## Simulator Part

### Agent

`Agent` is the basic class of the vehicle in the simulator. **You may not use it directly.**

```C++
Agent(int id, Vector initialState); // constructor

void setState(Vector st);
void setNextState(Vector state);
void setPreState(Vector state);	// push the state into historical states
void applyNextState();

void setfollowingPlanner(Planner *p);	// set the planner for normal driving
void setlinechangePlanner(Planner *p); // set the planner for line change action
void setController(Controller *c);	// set the controller
void setModel(Model *m);	// set the model this agent use
void setBehaviour(Behaviour *b); // set the Behavior model this agent use
void setMapinfo(MapInfo *m);		// set the MapInfo which is the reference manager
void setEgoCar();	// set the car as the ego car
void setInPredictor(Predictor *p);	// set the internal predictor
void setExPredictor(Predictor *p); // set the external predictor

int getId() const;	// get the id of this agent
Vector getState() const;	// get the current state
std::vector<Vector>  get_preState() const;	// get the historical states

virtual AgentType getType() const = 0;	// get the type of this agent
bool isEgoCar() const;	// get whether the car is the ego car
Predictor* getInPredictor();
Predictor* getExPredictor();


void Run(); // main function, run this car at this time step `
```

Agent Types:

```C++
enum AgentType {	//Now we only use ReplayCar & BehaveCar
    HumanCar = 0,
    VirtualCar = 1,
    RealCar = 2,
    ReplayCar = 3,
    NewVirtualCar,
    BehaveCar
};
```

### BehaveCar

`BehaveCar` is the class of the ROBOT vehicle in the simulator. 

```C++
BehaveCar(int id, std::vector<double> initialState); // Constructor

bool IDM_; // Special variable. If false, the agent will use planner it set to do the planning. If true, it will just use IDM model.
```

### ReplayAgent 

`ReplayAgent` is the class of the replay vehicle in the simulator.  (see `core/my_impl/Behavious/ReplayGenerator.cpp`)

```C++
ReplayAgent(int id, Vector initialState); // Constructor

void setTrajectory(Trajectory traj); // MUST call this function before Run()
const Trajectory getTrajectory(); // get trajectory

void set_planner_buffer(); // set planner buffer for ground truth predictor
Vector Update(); // Get the next state
```

### Behaviour

`Behaviour` is the basic class of the behavior model in the simulator. **You may not use it directly.**  

```C++
Behaviour (Agent* agent_ibt, BehaviourType t); // Constructor, ibt means it belongs to

BehaviourType getType(); // get the type of naïve action algorithm this behavior use
Mode getMode();	// get the current mode of this Planner

void setMode(Mode new_mode);

std::vector<double> update(std::vector<double> currentState, const std::vector<double> &humanInput, std::vector<Agent*> agents); // main function, run this behavior at this time step
```

Behaviours Type:

```C++
enum BehaviourType {
    IDM = 0,
    OVM,
    GFM
};/// This enumerator stands for the type of an Behaviour.
```

Mode:

```C++
enum Mode {
    following = 0,
    merging,
    yielding,
    stopping,
    vanishing,
    synchronizing,
    allWayStopping,
    linechange,
    end,
    init,           //init
    stop2follow,    //there is a stopline within 10m, but a car is nearer than 10m.
};
```

Obstacle information:

```C++
struct Obstacle_info{
    Agent* agentptr;
    TraPoints point_in;
    double distence;
    bool yielding=true;
};
```

### AoBehaviour

`AoBehaviour` is the class of the rule based behavior model in the simulator. 

```C++
AoBehaviour(Agent* agent_ibt, BehaviourType t); //Constructor

Vector Following(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
Vector Stopping(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);

void getObstacle(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents); // get obstacles

Vector get_stop_plan(double remaining, double v_init, double a_init);
Vector interpolate_stop(Vector plan, double t_new);


Vector update(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents); // main function
```

### ReplayGenerator 

`ReplayGenerator` is the class to load csv and get replay infomation

```C++
void loadCSV(std::string filePath); /// Load data from recorded CSV file

std::vector<std::vector<int> > filter_replay_car(int ReplayStartTimestamp_ms, int ReplayEndTimestamp_ms); // Get the replay pool from [ReplayStartTimestamp_ms, ReplayEndTimestamp_ms]

void pre_load_state(Agent* virtualCar, int track_id, int start_timestamp, int end_timestamp); // load historical trajectory (for better prediction)

Trajectory elicit_trajectory(int track_id, int start_timestamp, int end_timestamp); // elicit the trajectory of track_id from [start_timestamp, end_timestamp]
```

### Planner

`Planner` is the basic class of the planner in the simulator. **You may not use it directly.**

```C++
Planner(Agent* agent_ibt, int dimState, int dimInput, MapInfo *map = nullptr); //Constructor

PlannerType getType(); // get the type of this Planner

void updatepre(PlannerPre& new_pre); // update the parameter of this planner. [not implement now and will fix the typo(updatepre->updatepara; PlannerPre->PlannerPara) in the future]


std::vector<double> update(std::vector<double> currentState, const std::vector<double> &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info); // main function, run this planner at this time step
```

Planner types:

```C++
enum PlannerType {
    Astar = 0,
    EB,
    CILQR
};
struct AstarPre{
    float w_a=0;
    float w_an=0;
    float w_v=1;
};
struct EBPre{

};
struct CILQRPre{

};
struct PlannerPre
{
    AstarPre AstarPre_;
    EBPre EBPre_;
    CILQRPre CILQRPre_;
};
```

### AstarPlanner 

`AstarPlanner` is the class of the A\* search based planner in the simulator. 

```C++
AstarPlanner(Agent* agent_ibt, MapInfo* map); //Constructor

Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info); // main function
```

### Predictor

`Predictor`  is the basic class of the Predictor in the simulator. **You may not use it directly.**  

```C++
Predictor (Agent* agent_ibt, double time_step, double horizon); // Constructor

virtual PredictorType getType(); // get the type of this Predictor
virtual void set_traj(PredictTra traj) = 0; // set the predicted results from the client (only if it is the py_predictor)

PredictorState get_state(); // the state of the predictor
void set_state(PredictorState s);

virtual PredictTra update(std::vector<double> currentState, std::vector<Agent*> agents); // main function, run this Predictor at this time step
```

Predictor Types:

```C++
enum PredictorType {
    ConstantSpeedPredictor = 0,
    GroundTruthPredictor = 1,
    NoPredictor = 2,
    PyPredictor = 3,
};
```

Predictor States:

```C++
enum PredictorState {
    fine = 0,           // ready for the next update time
    wait4fetch = 1,     // the py_predictor is waiting, and the historical trajectory hasn't been sent to the client
    wait4upload = 2,    // the py_predictor is waiting, and the historical trajectory has been sent to the client
    wait4update = 3,    // the prediction has done

    // NOTE: If the predictor is not a py_predictor, directly set the state as `wait4update` when going into `xxPredictor::update()`
};
```

Data structures of trajectories and points:

```C++
struct TraPoints // the data type of points used in the simulator
{
    double t;
    double x;
    double y;
    double theta;
    double delta_theta;
    double v;
    double a;
    double jerk;
    ConstLanelet current_lanelet;
    double s_of_current_lanelet;
    double d_of_current_lanelet;
};
struct OneTra
{
    std::vector<TraPoints> Traj;
    std::vector<ConstLanelet> confilictlanes;
    double Probability;
};
struct PredictTra
{
    std::vector<OneTra> Trajs;
};
```

### GroundTruthPredictor 

`GroundTruthPredictor` is the class of predictor that will give you the ground truth of the vehicle directly. 

```C++
GroundTruthPredictor (Agent* agent_ibt, double time_step, double horizon); // Constructor
```

### ConstantSpeedPredictor 

`ConstantSpeedPredictor` is the class of predictor that use constant velocity to predict vehicle stats. 

```C++
ConstantSpeedPredictor (Agent* agent_ibt, double time_step, double horizon); // Constructor
```

### PyPredictor 

`PyPredictor` is the class of predictor that use an external python program to do the prediction through GRPC.

```C++
PyPredictor (Agent* agent_ibt, double time_step, double horizon); // Constructor

PredictTra ClientTraj; // the special variable to store the results from the python side
```

### NoPredictor

`NoPredictor` is the class of the predictor that assigns to the `replay car without prediction`.

```C++
NoPredictor(Agent* agent_ibt, double time_step, double horizon); // Constructor
```

### Mapinfo 

`Mapinfo` is the class containing the map and reference information.

```C++
MapInfo(LaneletMapPtr& mapPtr, routing::RoutingGraphPtr& rgPtr); //Constructor
void init(int id, Vector initstate); // set a init states of the agent you may use it after get the routing path

bool setRoutingPath(ConstLanelet& startLanelet, ConstLanelet& destinationLanelet); // get the routing path from lanelet
void setLaneletPath(ConstLanelets& lanelet_path); // set the routing path by hand

void setCurrentLanelet(ConstLanelet& ll);
ConstLanelet getCurrentLanelet() { return currentLanelet_;}; // get the current lanelet where the agent in.

ConstLanelet getStopLanelet(); // return the closet lanelet containing a stopline, 
double getStopLaneletDis(); // return the distance to the closet lanelet containing a stopline


void update(Vector nextstate); //the main function, use the next state to update 
```

### Simulator

`Simulator` is the class doing the main loop (see `core/my_impl/Simulator/Simulator.hpp`).

```C++
Simulator(int rviz_port); // You can set rviz point for real time visualization, or set rviz_port = -1 to avoid that visualization

void generateReplayCar(ReplayCarInfo replay_info); // Generate Replay Car according to replay_info
void generateBehaveCar(ReplayCarInfo behave_info); // Generate Behavior Cars according to replay_info

bool removeAgentIfNeeded(); // remove the cars that have arrived at the terminal point.
void Agentmanager(); // generate cars at each timestep

int collisionWithLane(double x[], double y[],  ConstLineString2d left, ConstLineString2d right, double xx, double yy, double Thre);
void isThereCollision(); // check whether there is any collision

void LogTick(); // output to log file
void updateTick(); // update the states for all the cars 

//For gRPC
core::Trajectory ToTraj(Agent* agent); // Get historical trajectory of agent
core::SimulationEnv fetch_history(); // find a car that is waiting for python predictor.
void upload_traj(int car_id, std::vector<core::Trajectory> pred_trajs, std::vector<double> probability); // upload the results from the python side.


void InitSimulation(std::string scenario_id, std::string Config_Path, std::string log_folder, const bool verbose); // load config

void run(); // main function, start simulation
```

Data Structures:

```C++
typedef std::tuple<int, int, int, Vector, std::string> ReplayCarInfo;
//ReplayCarInfo: (track_id, start_ms, end_ms, init_state, others)

struct State {
    uint64_t track_id;
    uint64_t frame_id;
    uint64_t timestamp_ms;
    std::string agent_type;
    double x;
    double y;
    double vx;
    double vy;
    double psi_rad;
    double length;
    double width;
    double jerk;
    uint64_t current_lanelet_id;
    double s_of_current_lanelet;
    double d_of_current_lanelet;
};

typedef std::vector<State*> Trajectory;

struct SimulationEnv {  // the input of the client
    Trajectory my_traj;
    std::string map_name;
    std::vector<Trajectory> other_trajs;

    bool paused;    // whether the simulator has paused.
};
```

## gRPC Part

### MySimulator

`MySimulator` is the **abstract** class that wrapper the above simulator. (See `core/simulator.hpp`).

```C++
virtual void start(const SimulationScenario &scenario, const std::string &config_file, const std::string &log_folder, const bool verbose) = 0; // main function, use another thread to start the simulator

virtual bool onUserState(std::vector<Trajectory> pred_trajs, std::vector<double> probability) = 0; // upload results from the client
virtual SimulationEnv fetchEnv() = 0; // get inputs for the client

virtual void shutdown() = 0;
```

### MySimulatorImpl

`MySimulatorImpl` is the implemented class derivated from `MySimulator`. (See `core/my_impl/my_simulator_impl.hpp`).

```C++
MySimulatorImpl(int rviz_port); // constructor

Simulator simulator; // special variable
```

### ServiceImpl

`ServiceImpl` is the class used to communicate with the client. (See `service/impl/service_impl.h`)

```C++
ServiceImpl(core::MySimulator* simulator); // constructor

static void TrajToProtoTraj(core::Trajectory& coreTraj, service::Trajectory* protoTraj); // convert `core::Trajectory` to `service::Trajectory`

static void ProtoTrajToTraj(const service::Trajectory& protoTraj, core::Trajectory* coreTraj); // convert `service::Trajectory` to `core::Trajectory`

grpc::Status FetchEnv(grpc::ServerContext *, const service::FetchEnvRequest *, service::FetchEnvResponse *); // fetch the input of client

grpc::Status PushMyTrajectory(grpc::ServerContext *, const service::PushMyTrajectoryRequest *, service::PushMyTrajectoryResponse *); // upload the prediction results

core::MySimulator* m_simulator; // special variable
```

### Service

`Service` is the class doing the main loop. (See `service/service.h`).

```C++
Service(core::MySimulator* simulator);

int run(core::SimulationScenario& scenario, std::string address, int port, const std::string &config_file, const std::string &log_folder, const bool verbose); // do the main loop

ServiceImpl* m_impl;    // Run communication
core::MySimulator *m_simulator; // Run simulator
```

## Proto

The message types are defined in `../proto/simulator.proto`. You can modify it if you want to send more information.

Here are some tips

- `rpc FetchEnv (FetchEnvRequest) returns (FetchEnvResponse) {}` is used to fetch inputs for the client. 

  ```protobuf
  // request & response for fetching environment from remote server
  message FetchEnvRequest{
  }
  
  message FetchEnvResponse {
      int32 resp_code = 1;
      string msg = 2;
  
      string map_name = 3; // string, like FT, OF, MA
      Trajectory my_traj = 4; //the historical trajectory of the ego car
      repeated Trajectory other_trajs = 5; //the historical trajectories of the surrounding cars.
  }
  ```

- `rpc PushMyTrajectory (PushMyTrajectoryRequest) returns (PushMyTrajectoryResponse) {}` is used to upload results from the client.

  ```protobuf
  // request & response for pushing my trajectory to remote server
  message PushMyTrajectoryRequest {
      repeated Trajectory pred_trajs = 1; // predicted trajectories
      repeated double probability = 2;  // the probability for each trajectory. Sum(probability) = 1.0
  }
  
  message PushMyTrajectoryResponse {
      int32 resp_code = 1;
      string msg = 2;
  }
  ```

  



