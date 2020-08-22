//
// Created by mscsim on 8/24/18.
//

#include "Server.hpp"

#include <json/json.h>
#include <json/reader.h>
#include <json/writer.h>

#include <iostream>
#include <sstream>
#include <fstream>
double PI = 3.1415926535;

/// Respond to a request from a client for rendering in UE4
/// \param requestJson request string in Json format
/// \return response string in Json format
string Server::respondRendering(const string &requestJson) {

    Json::Reader reader;
    Json::Value root;

    Json::Value ret;

//    std::cout << "Respond Rendering : " << requestJson << std::endl;
    try {

        bool isParsingSuccessful = reader.parse(requestJson, root);

        if (isParsingSuccessful && root["sender"].asString() == "rendering") {
            ret["respondStatus"] = true;

            string command = root["command"].asString();

            if (command == "newSession") { // if the client connects for the first time, and requests a new session

                mutex.lock();

                int sessionId = this->sessionIdCounter++;
                ret["sessionId"] = sessionId;

                Session *session = new Session();
                session->sessionId = sessionId;
                session->sessionType = SessionType::Rendering;

                this->sessionAgentMap[session] = nullptr;

                mutex.unlock();

                std::cout << "request new session for rendering" << std::endl;

            } else if (command == "getAgents") { // if the client already has a session, and gets the agent information.

                mutex.lock();

                int sessionId = root["sessionId"].asInt();
                Session *session = this->findSessionById(sessionId);

                int targetAgentId =
                        (this->sessionAgentMap.find(session)->second == nullptr) ?
                        (-1) : (this->sessionAgentMap.find(session)->second->getId());
                // return -1 if the session is not bound to any agent


                Json::Value agentInfo = this->getAgentInfo(false);

                mutex.unlock();

                ret["agents"] = agentInfo;
                ret["targetAgentId"] = targetAgentId;

                // std::cout << "request agent info" << std::endl;
            } else if (command == "getPedestrianAgents") {
                mutex.lock();
  
                int sessionId = root["sessionId"].asInt();
                Session *session = this->findSessionById(sessionId);
                /*
                int targetAgentId =
                        (this->sessionAgentMap.find(session)->second == nullptr) ?
                        (-1) : (this->sessionAgentMap.find(session)->second->getId());
                // return -1 if the session is not bound to any agent
                */
                //int targetAgentId = 5;
  
                Json::Value pedestrianAgentInfo = this->getPedestrianAgentInfo(false);
  
                mutex.unlock();
  
                ret["pedestrianAgents"] = pedestrianAgentInfo;
                //ret["targetAgentId"] = targetAgentId;
                //ret["agents"] = "Hello Pedestrian!!";
                //ret["targetAgentId"] = "Pedestrian ID";
            } else if (command == "updateInput") {
                mutex.lock();

                Vector input = Json2Vector(root["input"]);
                int sessionId = root["sessionId"].asInt();
                Session *session = this->findSessionById(sessionId);
                PedestrianAgent *pedestrianAgent = this->findPedestrianAgentById(0);

                this->humanInputs[pedestrianAgent] = input;
                std::string writebuf = " ";
                        writebuf += std::to_string(0) + ' ' + std::to_string(input[0] / 100.0) + ' ' + std::to_string(input[1] / 100.0) + ' ' +
                                    std::to_string((PI*input[2]/180)) + ' ' + '\n';
                writebuf = std::to_string(1) + writebuf;

                std::ofstream recordPedestrians;
                recordPedestrians.open("/home/mscsim/mkz-mpc-control/Pedestrians.txt", std::ios::trunc | std::ios::out);
                if (recordPedestrians.is_open()) {
                    recordPedestrians << writebuf;
                    recordPedestrians.close();
                }
                else
                {
                    std::cout << "no such file" << std::endl;
                }
                Json::Value agentInfo = this->getPedestrianAgentInfo(false);
                mutex.unlock();
                ret["pedestrianAgents"] = agentInfo;
                //ret["targetAgentId"] = targetAgentId;
                //ret["agents"] = "Hello Pedestrian!!";
                //ret["targetAgentId"] = "Pedestrian ID";
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
    //std::cout << output << std::endl;

    return output;
}
