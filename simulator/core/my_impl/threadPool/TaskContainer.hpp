//
// Created by m-1 on 10/24/18.
//

#ifndef UNTITLED_TASKCONTAINER_H
#define UNTITLED_TASKCONTAINER_H


#pragma once
#include <queue>
class Task;
/// The class of task container
/** It contains task that need to be assigned. */
class TaskContainer
{
public:
    TaskContainer();
    ~TaskContainer();
    void push(Task *);
    Task* top();
    void pop();
    std::priority_queue<Task*>::size_type size();
private:
    std::priority_queue<Task*> task_container_;  /*!< The task container itself*/
};




#endif //UNTITLED_TASKCONTAINER_H
