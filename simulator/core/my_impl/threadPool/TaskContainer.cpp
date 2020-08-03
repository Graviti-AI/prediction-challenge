//
// Created by m-1 on 10/24/18.
//

#include "TaskContainer.hpp"


TaskContainer::TaskContainer()
{
}


TaskContainer::~TaskContainer()
{
}

///
/// Push a task into the task container.
void TaskContainer::push(Task* t)
{
    task_container_.push(t);
}

///
/// Get the top task from the task container.
Task* TaskContainer::top()
{
    return task_container_.top();
}

///
/// Get the top task from the task container.
void TaskContainer::pop()
{
    task_container_.pop();
}

///
/// Get the number of tasks in the task container.
std::priority_queue<Task*>::size_type  TaskContainer::size()
{
    return task_container_.size();
}
