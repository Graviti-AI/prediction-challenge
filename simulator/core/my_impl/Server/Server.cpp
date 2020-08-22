//
// Created by mscsim on 8/23/18.
//

#include "Server.hpp"
#include <json/json.h>
#include <json/reader.h>
#include <json/writer.h>

///
/// \param port port for simulation.
/// \param simulatorState reference to the simulator state, an enumerator.
/// \param humanInputs reference to a map from agent to pertaining vector human input.
/// \param agentDictionary reference to a map from agent to pertaining controller, model, and planner.
/// \param mutex reference to a global mutex lock, which avoids thread conflicts.
Server::Server(int port, SimulatorState &simulatorState, InputDictionary &humanInputs, AgentDictionary &agentDictionary, PedestrianAgentDictionary &pAgentDictionary, std::mutex &mutex)
: server(port), simulatorState(simulatorState), humanInputs(humanInputs), agentDictionary(agentDictionary), mutex(mutex), pAgentDictionary(pAgentDictionary) {
    this->agentIdCounter = 0;
    this->sessionIdCounter = 0;

    this->server.bind("requestManager", [this](string requestJson){
       return this->respondManager(requestJson); // if the request was sent by a manager
    });

    this->server.bind("requestRendering", [this](string requestJson){
        return this->respondRendering(requestJson); // if the request was sent by a rendering client
    });

    this->server.bind("requestInput", [this](string requestJson){
        return this->respondInput(requestJson); // if the request was sent by a client for hardware input
    });

    this->server.async_run(10); // Run the server async with not more than 5 threads
//    this->server.run();
}

/// Convert a std::vector<double> to a Json::Value
/// \param val a json value (defined in jsoncpp library)
/// \return a STL vector
Vector Server::Json2Vector(const Json::Value &val) {
    Vector vec;
    for (auto iter = val.begin(); iter != val.end(); iter++) {
        vec.push_back(iter->asDouble());
    }
    return vec;
}

/// Convert a Json::Value to a std::vector
/// \param vec a STL vector
/// \return a json value (defined in jsoncpp library)
Json::Value Server::Vector2Json(const Vector &vec) {
    Json::Value val;
    for (double temp : vec) {
        val.append(Json::Value(temp));
    }
    return val;
}

/// Get the information of all agents in the simulator
/// \param withLock should use mutex lock or not
/// \return agent information serialized in Json format
Json::Value Server::getAgentInfo(bool withLock) const {

    Json::Value ret(Json::arrayValue);

    if (withLock) {
        this->mutex.lock();
    }

    for (auto pair : this->agentDictionary) {

        Agent *agent = pair.first;

        Json::Value temp;

        temp["id"] = agent->getId();

        switch (agent->getType()) {
            case HumanCar:
                temp["type"] = "HumanCar";
		temp["input"] = Vector2Json(humanInputs.at(agent));
                break;
            case VirtualCar:
                temp["type"] = "VirtualCar";
                break;
            case RealCar:
                temp["type"] = "RealCar";
                break;
        }

        temp["state"] = Vector2Json(agent->getState());

        ret.append(temp);
    }

    if (withLock) {
        this->mutex.unlock();
    }


    return ret;
}

/// Get the information of all pedestrian agents in the simulator
/// \param withLock should use mutex lock or not
/// \return pedestrian agent information serialized in Json format
Json::Value Server::getPedestrianAgentInfo(bool withLock) const {
    Json::Value ret(Json::arrayValue);

    if (withLock) {
        this->mutex.lock();
    }
//    std::cout << "Ped num : " << this->pAgentDictionary.size() << std::endl;
    for (auto pair : this->pAgentDictionary) {

        PedestrianAgent *pedestrianAgent = pair.first;

        Json::Value temp;

        temp["id"] = pedestrianAgent->getId();

        temp["state"] = Vector2Json(pedestrianAgent->getState());

        ret.append(temp);
    }

    if (withLock) {
        this->mutex.unlock();
    }

    return ret;
}

/// Get the information of all sessions in the simulator
/// \param withLock should use mutex lock or not
/// \return agent information serialized in Json format
Json::Value Server::getSessionInfo(bool withLock) const {

    Json::Value ret(Json::arrayValue);

    if (withLock) {
        this->mutex.lock();
    }


    for (SessionAgentPair pair : this->sessionAgentMap) {

        Session *session = pair.first;

        Json::Value temp;

        temp["id"] = session->sessionId;

        switch (session->sessionType) {
            case SessionType::Rendering:
                temp["type"] = "Rendering";
                break;
            case SessionType::Input:
                temp["type"] = "Input";
                break;
        }

        temp["boundToAgentId"] =
                (pair.second == nullptr) ?
                (-1) : (this->sessionAgentMap.find(session)->second->getId());

        ret.append(temp);
    }

    if (withLock) {
        this->mutex.unlock();
    }

    return ret;


}

/// Find an agent in the simulator by its id.
/// \param id id of the agent to find
/// \return pointer to that agent
Agent *Server::findAgentById(int id) const {

    for (auto pair : this->agentDictionary) {
        Agent *agent = pair.first;
        if (agent->getId() == id) {
            return agent;
        }
    }
    throw std::runtime_error("bad id"); // throw runtime error if not found (should be caught by simulator)
}

/// Find a pedestrianAgent in the simulator by its id.
/// \param id id of the agent to find
/// \return pointer to that agent
PedestrianAgent *Server::findPedestrianAgentById(int id) const {

    for (auto pair : this->pAgentDictionary) {
        PedestrianAgent *pedestrianAgent = pair.first;
        if (pedestrianAgent->getId() == id) {
            return pedestrianAgent;
        }
    }
    throw std::runtime_error("bad id"); // throw runtime error if not found (should be caught by simulator)
}


/// Find a session in the simulator by its id. Session is an abstraction of a connection socket to a client.
/// \param id id of the session to find
/// \return pointer to that session
Server::Session *Server::findSessionById(int id) const {

    Session *ret = nullptr;
    for (SessionAgentPair pair : this->sessionAgentMap) {

        Session *session = pair.first;

        if (session->sessionId == id) {
            return session;
        }
    }
    throw std::runtime_error("bad id"); // throw runtime error if not found (should be caught by simulator)
}
