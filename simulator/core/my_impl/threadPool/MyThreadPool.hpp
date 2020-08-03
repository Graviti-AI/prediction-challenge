//
// Created by m-1 on 10/24/18.
//

#ifndef UNTITLED_MYTHREADPOOL_H
#define UNTITLED_MYTHREADPOOL_H


#pragma once

#include <thread>
#include <mutex>
#include "Task.hpp"
#include "MyThread.hpp"
#include "BusyThreadContainer.hpp"
#include "IdleThreadContainer.hpp"
#include "TaskContainer.hpp"

class MyThread;
/// The class of the thread pool
/** It manages all thread for calculating agents. */
class MyThreadPool
{
public:

    MyThreadPool(){}
    MyThreadPool(int number);
    ~MyThreadPool();
    void AddTask(Task *Task,int priority);
    void AddIdleThread(int n);
    void RemoveThreadFromBusy(MyThread *myThread);
    void Start();
    void EndMyThreadPool();
private:
    BusyThreadContainer busy_thread_container_; /*!< Container for threads which is not in using.*/
    IdleThreadContainer idle_thread_container_; /*!< Container for threads which is in using.*/
    bool issurvive_; /*!< Flag for whether the thread pool is in using.*/
    TaskContainer task_container_; /*!< Container for tasks.*/
    std::thread thread_this_; /*!< The thread for the thread pool manage all threads.*/
    std::mutex busy_mutex_; /*!< Mutex for busy_thread_container_. */
    std::mutex idle_mutex_; /*!< Mutex for idle_thread_container_. */
    std::mutex task_mutex_; /*!< Mutex for task_container_. */
    int number_of_thread_;  /*!< Number of threads in idle_thread_container_ which means the maximum threads the thread pool can use simultaneously. */
};



#endif //UNTITLED_MYTHREADPOOL_H
