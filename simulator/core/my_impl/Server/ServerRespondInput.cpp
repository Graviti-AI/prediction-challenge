//
// Created by mscsim on 8/24/18.
//

#include "Server.hpp"

#include <json/json.h>
#include <json/reader.h>
#include <json/writer.h>
#include <iostream>

/// Respond to a request from a client for input hardware
/// \param requestJson request string in Json format
/// \return response string in Json format
string Server::respondInput(const string &requestJson) {

    Json::Reader reader;
    Json::Value root;

    Json::Value ret;

//     std::cout << "respond Input : " << requestJson << std::endl;

    try {

        bool isParsingSuccessful = reader.parse(requestJson, root);

        if (isParsingSuccessful && root["sender"].asString() == "input") {
            ret["respondStatus"] = true;

            string command = root["command"].asString();





            if (command == "newSession") { // if the client connects for the first time, and requests a new session

                mutex.lock();

                int sessionId = this->sessionIdCounter++; // Allocate a new session id
                ret["sessionId"] = sessionId;



                Session *session = new Session();
                session->sessionId = sessionId;
                session->sessionType = SessionType::Input;

                this->sessionAgentMap[session] = nullptr;

                mutex.unlock();

            } else if (command == "updateInput") { // if the client already has a session, and updates the input vector.

                mutex.lock();

                Vector input = Json2Vector(root["input"]);
                int sessionId = root["sessionId"].asInt(); // the client should provide its previous session id

                Session *session = this->findSessionById(sessionId);
                Agent *agent = this->sessionAgentMap[session];

                this->humanInputs[agent] = input;

                mutex.unlock();
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