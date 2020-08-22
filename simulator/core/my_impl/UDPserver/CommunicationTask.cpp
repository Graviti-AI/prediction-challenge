//
// Created by lcr on 2/23/20.
//

#include "CommunicationTask.h"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> /* struct hostent */
#include <arpa/inet.h> /* inet_ntop */
#include "unistd.h"
#include <string.h>
#define SERVER_PORT 8086
#define BUFF_LEN 60000



CommunicationTask::CommunicationTask(uint32_t ipp ,int current_fd, int id,  string &IP, u_int16_t &port_id, std::mutex &mutex,AgentDictionary &agentDictionary):ipp_(ipp),the_fd_(current_fd),ID_(id),IP_(IP), p_id(port_id),mutex(mutex),counter(0), agentDictionary(agentDictionary)
{}

void CommunicationTask::Run() {
    int server_fd, ret;
    struct sockaddr_in ser_addr;

    server_fd = the_fd_; //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        return;
    }
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = ipp_;
    ser_addr.sin_port = p_id;
    // ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    // if (ret < 0) {
    //     std::cout << "Bind failed " << std::endl;
    // }
    // std::cout << "@@@@ clent_addr : " << ser_addr.sin_addr.s_addr << std::endl; 
    // std::cout << "@@@@ clent_addr port: " <<  ser_addr.sin_port << std::endl; 
    std::cout << "@@@@ file desc  " << server_fd << std::endl;
    while (true){
        this->udp_msg_sender(server_fd, (struct sockaddr*)&ser_addr);
        usleep(1e6 * 0.1);
        //std::cout << "Count  ~" << counter << std::endl;
        my_mutex.lock();
        if (counter>3000) break;
        my_mutex.unlock();
    }
    std::cout << "Out ~" << std::endl;
    return;
}

void CommunicationTask::udp_msg_sender(int fd, struct sockaddr* dst)
{

    socklen_t len;
    Json::Value ret;
    ret["agentInfo"] = this->getAgentInfo();
    
    Json::FastWriter fastWriter;
    std::string output = fastWriter.write(ret);
    // std::cout << "Size 1 : " << output.size() << std::endl;
    // std::cout << "Content : " << output << std::endl;
    // output = "awsl";
    // std::cout << "Size 2 : " << output.size() << std::endl;
    // std::cout << "Content : " << output << std::endl;

    len = sizeof(*dst);
    int ee;
    ee = sendto(fd, output.c_str(), BUFF_LEN, 0, dst, len);
    if (ee < 0) {
        char buffer[ 25600 ];
        char * errorMsg = strerror_r( errno, buffer, 256 ); // GNU-specific version, Linux default
        printf("Error %s", errorMsg);
        return;
    }

    counter++;
}

Json::Value CommunicationTask::getAgentInfo() {

    Json::Value ret(Json::arrayValue);

    this->mutex.lock();

    for (auto pair : this->agentDictionary) {

        Agent *agent = pair.first;

        Json::Value temp;

        temp["id"] = agent->getId();

        switch (agent->getType()) {
            case HumanCar:
                temp["type"] = "HumanCar";
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

    this->mutex.unlock();



    return ret;
}

/// Convert a std::vector<double> to a Json::Value
/// \param val a json value (defined in jsoncpp library)
/// \return a STL vector
Vector CommunicationTask::Json2Vector(const Json::Value &val) {
    Vector vec;
    for (auto iter = val.begin(); iter != val.end(); iter++) {
        vec.push_back(iter->asDouble());
    }
    return vec;
}

/// Convert a Json::Value to a std::vector
/// \param vec a STL vector
/// \return a json value (defined in jsoncpp library)
Json::Value CommunicationTask::Vector2Json(const Vector &vec) {
    Json::Value val;
    for (double temp : vec) {
        val.append(Json::Value(temp));
    }
    return val;
}