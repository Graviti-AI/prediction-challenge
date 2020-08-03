//
// Created by m-1 on 10/24/18.
//

#ifndef UNTITLED_MYTHREAD_H
#define UNTITLED_MYTHREAD_H


#pragma once

#include "Task.hpp"
#include <thread>


class MyThreadPool;
class Task;
/// The class of Mythread
/** It manage one thread */
class MyThread
{
    friend bool operator==(MyThread my1, MyThread my2);
    friend bool operator!=(MyThread my1, MyThread my2);
public:
    MyThread(MyThreadPool *pool);
    void Assign(Task *Task);
    void Run();
    void StartThread();
    int getthreadid();
    void setisdetach(bool isdetach);
private:
    MyThreadPool *mythreadpool_; /*!< The thread pool this thread signed in.*/
    static int  s_threadnumber;  /*!< An counter for Mythraed to get different id for thread ID*/
    bool isdetach_; /*!< Flag for thread whether detach or join.*/
    Task *task_;  /*!< The task that the thread will handle.*/
    int threadid_; /*!< Thread ID*/
    std::thread thread_; /*!< Thread itself*/
};







#endif //UNTITLED_MYTHREAD_H
