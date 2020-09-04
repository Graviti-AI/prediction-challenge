//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_SIMULATOR_HPP
#define AGENTSIM_SIMULATOR_HPP
#include <sys/time.h>
#include <mutex>
#include <string>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include "../Server/Server.hpp"
#include "../UDPserver/MyClientPool.h"

#include "../simulator_state.hpp"
#include "../Agents/Agent.hpp"
#include "../Agents/PedestrianAgent.hpp"
#include "../Agents/RealCar.hpp"
#include "../Agents/HumanCar.hpp"
#include "../Agents/VirtualCar.hpp"
#include "../Agents/NewVirtualCar.hpp"
#include "../Agents/BehaveCar.hpp"

// #include "../Agents/ReplayAgent.hpp"
#include "../Controllers/VirtualCarController.hpp"
#include "../Planners/VirtualCarPlanner.hpp"
#include "../Models/VirtualCarModel.hpp"
#include "../threadPool/MyThreadPool.hpp"
#include "../Maps/TrafficInfo.hpp"
#include "../Behaviours/ReplayGenerator.hpp"
#include "../Maps/LaneletMapReader.hpp"
#include "../Predictors/ConstantSpeedPredictor.hpp"
#include "../Predictors/PyPredictor.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

class Controller;
class Model;
class Planner;

using std::vector;
using std::string;

#include <map>
#include <tuple>
using std::map;
using std::tuple;

typedef std::map<Agent*, std::tuple<Controller*, Model*, Planner*>> AgentDictionary;
typedef std::map<PedestrianAgent*, std::tuple<Controller*, Model*, Planner*>> PedestrianAgentDictionary;
//typedef std::map<Agent*, Vector> InputDictionary;
typedef std::map<Task*, Vector> InputDictionary;
using std::ifstream;
/// The class of the whole simulator
/** It manage all agents and use the thread pool to calculate the next state of each agent */

class Simulator {
public:
    Simulator(int rviz_port);
    
    void generateJinningCar_Obstacles(int Obs_id);
    void generateVirtualCar();
    void generateFSMVirtualCar();
    void generateReplayCar();
    void generateBehaveCar();
    bool removeAgentIfNeeded();
    void run();
    static InputDictionary humanInputsForThread; /*!< Reference to a map from agent to pertaining vector human input that should be used for calculate this time.*/
    static vector<Agent*> agentsForThread; /*!< Reference to a vector for all agents' information that should be used for calculate this time.*/
    static AgentDictionary agentDictionaryForThread;
    static int flagForVirtualCar; /*!< Decide whether update virtual car, based on timeuse > 0.01?.*/
    static int managerForVirtualCar; /*!< Decide whether begin to add the virtual car.*/
    static ifstream infile; /*!< .txt file for record all agents states.*/
    static int updateTimes; /*!< Counter for how many times the simulator have updated*/
    static double time; /*!< Record the time that the simulator cost for last updating, which is used to get timeuse. */
    static TrafficInfoManager* trafficInfoManagerPtr;
    static ReplayGenerator* ReplayGeneratorPtr;
    static vector<ReplayAgent*> replayAgentDictionary;
    static LaneletMapReader* mapreader;

    core::Trajectory ToTraj(Agent* agent);
    core::SimulationEnv fetch_history();
    void upload_traj(int car_id, std::vector<core::Trajectory> pred_trajs, std::vector<double> probability);

private:
    SimulatorState simulatorState; /*!< Reference to the simulator state, an enumerator.*/
    AgentDictionary agentDictionary; /*!< Reference to a map from agent to pertaining controller, model, and planner.*/
    PedestrianAgentDictionary pAgentDictionary;
    std::mutex mutex; /*!< Reference to a global mutex lock, which avoids thread conflicts.*/
    InputDictionary humanInputs; /*!< Reference to a map from agent to pertaining vector human input.*/
    MyThreadPool myThreadPool; /*!< The thread pool for calculate all agents*/
    int lineNumber;
    timeval t1, t2; /*!< Timeval for getting time used for arrange task for all agents*/
    double timeuse = 0; /*!< Record the time used and reset to 0 when it over 0.01 */

    MyClientPool* myClientPool; // For rviz visualization
    Server* server; // For rviz visualization

    void Agentmanager();
    void updateTick();
    void reset();

    static vector<string> split(const string &str,const string &pattern)
    {
        //const char* convert to char*
        char * strc = new char[strlen(str.c_str())+1];
        strcpy(strc, str.c_str());
        vector<string> resultVec;
        char* tmpStr = strtok(strc, pattern.c_str());
        while (tmpStr != NULL)
        {
            resultVec.push_back(string(tmpStr));
            tmpStr = strtok(NULL, pattern.c_str());
        }
        delete[] strc;
        return resultVec;
    }

    bool CILQR_car_flag;
    int total_car_num;
};


#endif //AGENTSIM_SIMULATOR_HPP
