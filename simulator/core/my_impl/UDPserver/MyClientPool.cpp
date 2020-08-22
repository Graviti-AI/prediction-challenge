//
// Created by lcr on 2/22/20.
//

#include "MyClientPool.h"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> /* struct hostent */
#include <arpa/inet.h> /* inet_ntop */
#include "unistd.h"
#include <string.h>
using namespace std;

#define SERVER_PORT 8086
#define BUFF_LEN 10240

///
/// \param number reference to the number of threads in the thread pool.
MyClientPool::MyClientPool(int number, std::mutex &mutex, AgentDictionary &agentDictionary): mythreadpool_(number),mutex(mutex),agentDictionary(agentDictionary)

{
    issurvive_ = true;
    number_of_thread_ = number;
    thread_this_ =thread(&MyClientPool::Start, this);
    thread_this_.detach();
}
MyClientPool::~MyClientPool()
{

}

///
/// End the thread pool.
void MyClientPool::EndMyClientPool()
{
    issurvive_ =false;
}


///
/// Start the thread pool.
/// It will keep scan whether exists idle threads in idle_thread_container_
/// and invoke idle threads to handle tasks and put that thread into busy_thread_container_.
void MyClientPool::Start()
{
    while (1)
    {
           
    int server_fd, ret;
    struct sockaddr_in ser_addr;

    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        return;
    }
    std::cout << "file desc" << server_fd << std::endl;
    int enable = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        printf("setsockopt(SO_REUSEADDR) failed");
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ser_addr.sin_port = htons(SERVER_PORT);

    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        char buffer[ 25600 ];
        char * errorMsg = strerror_r( errno, buffer, 256 ); // GNU-specific version, Linux default
        printf("Error %s", errorMsg);
        // return;
    }

    handle_udp_msg(server_fd);

    //close(server_fd);
    
    }

}

///
/// \param requestJson check the request
std::string MyClientPool::respondRendering(const std::string &requestJson) {
    Json::Reader reader;
    Json::Value root;

    Json::Value ret;


    try {
        bool isParsingSuccessful = reader.parse(requestJson, root);

        if (isParsingSuccessful && root["sender"].asString() == "rendering") {
            ret["respondStatus"] = true;

            std::string command = root["command"].asString();

            if (command == "newSession") { // if the client connects for the first time, and requests a new session

                //mutex.lock();
                int sessionId = this->sessionIdCounter++;
                ret["sessionId"] = sessionId;
                std::string IP = root["IP"].asString();
                // 接下来用这个IP构建Task开始发，并且搞一个map存一下id与这个Task
                class CommunicationTask *newtask = new class CommunicationTask(this_ipp,current_fd,sessionId, IP, cur_port_id, mutex, agentDictionary);
                auto pair = std::pair<int, CommunicationTask*>(sessionId, newtask);
                this->mythreadpool_.AddTask(newtask, 10);
                this->sessionDictionary_.insert(pair); // add the car to the simulator
                //mutex.unlock();

                std::cout << "request new session" << std::endl;

            } else if (command == "continue"){
                int sessionId = root["sessionId"].asInt();
                auto this_task = this->sessionDictionary_.at(sessionId);
                this_task->my_mutex.lock();
                this_task->counter = 0;
                this_task->my_mutex.unlock();
                ret["sessionId"] = sessionId;

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


void MyClientPool::handle_udp_msg(int fd)
{
    char buf[BUFF_LEN];
    socklen_t len;
    int count;
    struct sockaddr_in clent_addr;  //clent_addr用于记录发送方的地址信息
        memset(buf, 0, BUFF_LEN);
        len = sizeof(clent_addr);
        count = recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);
        if(count == -1)
        {
            printf("recieve data fail!\n");
            return;
        }

        printf("client: %s\n",buf);
        std::string requestJson;
        std::string sendJson;
        requestJson += buf;
        current_fd = fd;
        cur_port_id = clent_addr.sin_port;
        this_ipp = clent_addr.sin_addr.s_addr;

        sendJson = this->respondRendering(requestJson);
        std::cout << "clent_addr : " << this_ipp << std::endl; 
        std::cout << "clent_addr port: " << clent_addr.sin_port << std::endl; 
        std::cout << "file desc" << fd << std::endl;

        clent_addr.sin_family = AF_INET;
        // clent_addr.sin_addr.s_addr = 2189004298;
        
        // clent_addr.sin_port = htons(SERVER_PORT);
        sendto(fd, sendJson.c_str(), BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);
        
        // close(fd);
        std::cout<<sendJson<<std::endl;
    
}

