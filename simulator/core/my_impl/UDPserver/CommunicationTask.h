//
// Created by lcr on 2/23/20.
//

#ifndef AGENTSIM_COMMUNICATIONTASK_H
#define AGENTSIM_COMMUNICATIONTASK_H
#include <string>
#include <mutex>
#include "../threadPool/Task.hpp"
#include "../Agents/Agent.hpp"
#include "json/json.h"
#include <json/reader.h>
#include <json/writer.h>

using namespace std;
typedef std::map<Agent*, std::tuple<Controller*, Model*, Planner*>> AgentDictionary;

class CommunicationTask: public Task {

public:
    CommunicationTask(uint32_t ipp ,int current_fd,int id,  string &IP, u_int16_t &port_id, std::mutex &mutex,AgentDictionary &agentDictionary);
    void Run();
    void udp_msg_sender(int fd, struct sockaddr* dst);
    Json::Value getAgentInfo();
    static Vector Json2Vector(const Json::Value &val);
    static Json::Value Vector2Json(const Vector &vec);
    int the_fd_;
    int ID_;
    int counter;
    u_int16_t p_id;
    uint32_t ipp_;
    string IP_;
    AgentDictionary &agentDictionary;
    std::mutex &mutex;
    std::mutex my_mutex;
};


#endif //AGENTSIM_COMMUNICATIONTASK_H
