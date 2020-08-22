//
// Created by mscsim on 8/24/18.
//

#include "Server.hpp"

#include <json/json.h>
#include <json/reader.h>
#include <json/writer.h>

#include "../Agents/HumanCar.hpp"
#include "../Agents/RealCar.hpp"
#include "../Controllers/HumanCarController.hpp"
#include "../Models/HumanCarModel.hpp"
#include "../Planners/HumanCarPlanner.hpp"
#include "../Models/FourWheelModel.hpp"
#include "../Controllers/FourWheelController.hpp"
#include "../Planners/RealCarPlanner.hpp"
#include "../Models/RealCarModel.hpp"
#include "../Controllers/RealCarController.hpp"
#include "../Simulator/Simulator.hpp"

#include <iostream>

/// Respond to a request from a manager client, who controls the simulator server.
/// \param requestJson request string in Json format
/// \return response string in Json format
string Server::respondManager(const string &requestJson) {

    Json::Reader reader;
    Json::Value root;

    Json::Value ret;

//    std::cout << "Respond Manager : " << requestJson << std::endl;

    try {

        bool isParsingSuccessful = reader.parse(requestJson, root);

        if (isParsingSuccessful && root["sender"].asString() == "manager") {
            ret["respondStatus"] = true;

            string command = root["command"].asString();

            if (command == "pause") { // if the manager wants to pause the simulator
                this->mutex.lock();
                this->simulatorState = Paused;
                this->mutex.unlock();

            } else if (command == "start") { // if the manager wants to start the simulator
                this->mutex.lock();
                this->simulatorState = Running;
                //Simulator::flagForVirtualCar = 1;
                this->mutex.unlock();

            } else if (command == "reset") { // if the manager wants to reset the simulator
                this->mutex.lock();
                this->simulatorState = Reset;
                this->mutex.unlock();

            } else if (command == "startvc") {
                Simulator::managerForVirtualCar = 1;

            } else if (command == "getAgents") { // if the manager wants to get information of all agents
                ret["agentInfo"] = this->getAgentInfo();

            } else if (command == "getSessions") { // if the manager wants to get information of all sessions
                ret["sessionInfo"] = this->getSessionInfo();
            } else if (command == "getTimes"){
                ret["times"] = Simulator::updateTimes;
            } else if (command == "getTime"){
                ret["time"] = Simulator::time;
            } else if (command == "addAgent") { // if the manager wants to add an agent dynamically
                string agentType = root["agentType"].asString();

                if (agentType == "HumanCar") { // HumanCar has a kinematic model
                    this->mutex.lock();
                    int agentId = this->agentIdCounter++;
                    Vector initialState = Json2Vector(root["initialState"]);
                    class HumanCar *humanCar = new class HumanCar(agentId, initialState); // create a new car

                    std::tuple<Controller*, Model*, Planner*> temp(
                            new HumanCarController(), // create corresponding controller
                            new HumanCarModel(), // create corresponding model
                            new HumanCarPlanner() // create corresponding planner
                    );
                    auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(humanCar, temp);

                    this->agentDictionary.insert(pair); // add the car to the simulator
                    this->humanInputs.insert(std::pair<Agent*, Vector>(humanCar, Vector(3))); // add the car to the simulator
                    this->mutex.unlock();

                } else if (agentType == "FourWheelCar") { // FourWheelCar has a complex four-wheel model
                    this->mutex.lock();
                    int agentId = this->agentIdCounter++;
                    Vector initialState = Json2Vector(root["initialState"]);
                    class HumanCar *humanCar = new class HumanCar(agentId, initialState); // create a new car

                    std::tuple<Controller*, Model*, Planner*> temp(
                            new FourWheelController(), // create corresponding controller
                            new FourWheelModel(), // create corresponding model
                            new HumanCarPlanner() // create corresponding planner
                    );
                    auto pair = std::pair<Agent*, std::tuple<Controller*, Model*, Planner*>>(humanCar, temp);

                    this->agentDictionary.insert(pair); // add the car to the simulator
                    this->humanInputs.insert(std::pair<Agent*, Vector>(humanCar, Vector(3))); // add the car to the simulator
                    this->mutex.unlock();
                } else if (agentType == "RealCar") { // FourWheelCar has a complex four-wheel model
                    this->mutex.lock();
                    int agentId = this->agentIdCounter++;
                    Vector initialState = Json2Vector(root["initialState"]);
                    class RealCar *realCar = new class RealCar(agentId, initialState); // create a new car

                    std::tuple<Controller *, Model *, Planner *> temp(
                            new RealCarController(), // create corresponding controller
                            new RealCarModel(), // create corresponding model
                            new RealCarPlanner() // create corresponding planner
                    );
                    auto pair = std::pair<Agent *, std::tuple<Controller *, Model *, Planner *>>(realCar, temp);

                    this->agentDictionary.insert(pair); // add the car to the simulator
                    this->humanInputs.insert(
                            std::pair<Agent *, Vector>(realCar, Vector(3))); // add the car to the simulator
                    this->mutex.unlock();
                }


            } else if (command == "bindAgentToSession") { // if the manager wants to bind an agent to a session dynamically

                int agentId = root["agentId"].asInt(); // target agent id
                int sessionId = root["sessionId"].asInt(); // target session id

                this->mutex.lock();

                Agent *theAgent = nullptr;
                Session *theSession = this->findSessionById(sessionId);

                if (agentId < 0) {
                    theAgent = nullptr;
                    std::cout << "agent: " << (-1) << ", session: " << theSession->sessionId << std::endl;

                } else {
                    theAgent = this->findAgentById(agentId);
                    std::cout << "agent: " << theAgent->getId() << ", session: " << theSession->sessionId << std::endl;

                }

                if (this->sessionAgentMap.find(theSession) == this->sessionAgentMap.end()) {
                    std::cout << "not in the map" << std::endl;
                    this->sessionAgentMap.insert(std::pair<Session*, Agent*>(theSession, theAgent));
                } else {
                    std::cout << "in the map" << std::endl;
                    this->sessionAgentMap[theSession] = theAgent;
                }

                this->mutex.unlock();
            }

        } else {
            ret["respondStatus"] = false;
            ret["errorMessage"] = "bad json format";
        }

    } catch (const std::runtime_error &e) {
        ret["respondStatus"] = false;
        ret["errorMessage"] = e.what();
    } catch (...) {
        ret["respondStatus"] = false;
        ret["errorMessage"] = "unknown error";
    }


    Json::FastWriter fastWriter;
    std::string output = fastWriter.write(ret);

    return output;
}
