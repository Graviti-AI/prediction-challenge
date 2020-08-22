//
// Created by lcr on 2/22/20.
//

#ifndef AGENTSIM_MYCLIENTPOOL_H
#define AGENTSIM_MYCLIENTPOOL_H

#include <thread>
#include <mutex>
#include "json/json.h"
#include <json/reader.h>
#include <json/writer.h>
#include "../threadPool/Task.hpp"
#include "../threadPool/MyThread.hpp"
#include "../threadPool/BusyThreadContainer.hpp"
#include "../threadPool/IdleThreadContainer.hpp"
#include "../threadPool/TaskContainer.hpp"
#include "../threadPool/MyThreadPool.hpp"
#include "../Agents/Agent.hpp"
#include "CommunicationTask.h"
typedef std::map<Agent*, std::tuple<Controller*, Model*, Planner*>> AgentDictionary;
typedef std::map<int, CommunicationTask*> SessionDictionary;

using namespace std;

class MyClientPool {
public:

    MyClientPool(int number, std::mutex &mutex, AgentDictionary &agentDictionary);
    ~MyClientPool();
    void Start();
    void EndMyClientPool();
private:
    //string respondManager(const string &requestJson);
    string respondRendering(const string &requestJson);
    //string respondInput(const string &requestJson);
    void handle_udp_msg(int fd);
    SessionDictionary sessionDictionary_;
    AgentDictionary &agentDictionary;
    MyThreadPool mythreadpool_; /*!< Container for threads which is not in using.*/
    bool issurvive_; /*!< Flag for whether the thread pool is in using.*/
    std::thread thread_this_; /*!< The thread for the thread pool manage all threads.*/
    std::mutex &mutex; /*!< Mutex for task_container_. */
    int number_of_thread_;  /*!< Number of threads in idle_thread_container_ which means the maximum threads the thread pool can use simultaneously. */
    int sessionIdCounter = 0;  /*!< an counter for session to get different id for session*/
    int current_fd=-1;
    uint32_t this_ipp;
    u_int16_t cur_port_id;
};


#endif //AGENTSIM_MYCLIENTPOOL_H
