//
// Created by chenran on 2/20/20.
//

#include <string>
#include <vector>
#include <json/json.h>
#include <json/reader.h>
#include <json/writer.h>
#include <json/value.h>
#include <iostream>

/// Respond to a request from a client for rendering in UE4
/// it have two commands:
/// "newSession" if the client connects for the first time, and requests a new session.
/// "getAgents"  if the client already has a session, and gets the agent information.
/// \param requestJson request string in Json format
/// \return response string in Json format
string respondRendering(const string &requestJson) {
    Json::Reader reader;
    Json::Value root;

    Json::Value ret;


    try {

        bool isParsingSuccessful = reader.parse(requestJson, root);

        if (isParsingSuccessful && root["sender"].asString() == "rendering") {
            ret["respondStatus"] = true;

            string command = root["command"].asString();

            if (command == "newSession") { // if the client connects for the first time, and requests a new session

                //mutex.lock();

                int sessionId = 0;//this->sessionIdCounter++;
                ret["sessionId"] = sessionId;

                //Session *session = new Session();
                //session->sessionId = sessionId;
                //session->sessionType = SessionType::Rendering;

                //this->sessionAgentMap[session] = nullptr;

                //mutex.unlock();

                std::cout << "request new session" << std::endl;

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
