//
// Created by m-1 on 10/24/18.
//

#ifndef UNTITLED_TASK_H
#define UNTITLED_TASK_H


#pragma once

namespace
{
    enum  PRIORITY
    {

        MIN = 1, NORMAL = 25, MAX = 50
    };
}
/// The class of task
class Task
{

public:
    Task()
    {

    }
    /// set priority.
    void SetPriority(int priority)
    {
        if (priority>(PRIORITY::MAX))
        {
            priority = (PRIORITY::MAX);
        }
        else if (priority<(PRIORITY::MIN))
        {
            priority = (PRIORITY::MIN);
        }
    }
    virtual void Run();
protected:
    int priority_; /*!< The priority of this task. The range of weight is 1 to 50.*/
};



#endif //UNTITLED_TASK_H
