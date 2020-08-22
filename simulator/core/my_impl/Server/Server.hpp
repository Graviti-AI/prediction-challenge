//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_SERVER_HPP
#define AGENTSIM_SERVER_HPP

#include <mutex>
#include <string>
#include <vector>
//#include "../main.hpp"
//#include "../Simulator/Simulator.hpp"
#include "../simulator_state.hpp"
#include "../Agents/Agent.hpp"
#include "../Agents/PedestrianAgent.hpp"
#include "rpc/server.h"

using std::vector;
using std::string;

#include <map>
#include <json/value.h>

using std::map;

//typedef std::map<Agent*, Vector> InputDictionary;
typedef std::map<Task*, Vector> InputDictionary;


class Controller;
class Model;
class Planner;
typedef std::map<Agent*, std::tuple<Controller*, Model*, Planner*>> AgentDictionary;
typedef std::map<PedestrianAgent*, std::tuple<Controller*, Model*, Planner*>> PedestrianAgentDictionary;


/// The class of all server using rpclib.
/** It have three kinds of services for input, manager and rendering. */
class Server {

public:
    Server(int port, SimulatorState &simulatorState, InputDictionary &humanInputs,
	   AgentDictionary &agentDictionary, PedestrianAgentDictionary &pAgentDictionary, std::mutex &mutex);

private:
    string respondManager(const string &requestJson);
    string respondRendering(const string &requestJson);
    string respondInput(const string &requestJson);

    rpc::server server; /*!< server of rpclib*/
    SimulatorState &simulatorState; /*!< reference to the simulator state, an enumerator.*/
    AgentDictionary &agentDictionary; /*!< reference to a map from agent to pertaining controller, model, and planner.*/
    PedestrianAgentDictionary &pAgentDictionary;
    std::mutex &mutex; /*!< reference to a global mutex lock, which avoids thread conflicts.*/
    InputDictionary &humanInputs; /*!< reference to a map from agent to pertaining vector human input.*/

    static Vector Json2Vector(const Json::Value &val);
    static Json::Value Vector2Json(const Vector &vec);

/// An enum type
/// It represent to the session type.
    enum SessionType {
        Rendering = 0,
        Input = 1
    };
///
/// An structure for session including session id and its type.
    struct Session {
        int sessionId;
        SessionType sessionType;
    };

    typedef std::pair<Session*, Agent*> SessionAgentPair;

    map<Session*, Agent*> sessionAgentMap;
    Json::Value getSessionInfo(bool withLock = true) const;
    Json::Value getAgentInfo(bool withLock = true) const;
    Json::Value getPedestrianAgentInfo(bool withLock = true) const;

    int agentIdCounter; /*!< an counter for agent to get different id for agents*/
    int sessionIdCounter;  /*!< an counter for session to get different id for session*/

    Agent *findAgentById(int id) const;
    PedestrianAgent *findPedestrianAgentById(int id) const;
    Session *findSessionById(int id) const;
};


#endif //AGENTSIM_SERVER_HPP
