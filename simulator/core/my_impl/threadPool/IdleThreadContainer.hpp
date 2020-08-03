//
// Created by m-1 on 10/24/18.
//

#ifndef UNTITLED_IDLETHREADCONTAINER_H
#define UNTITLED_IDLETHREADCONTAINER_H


#pragma once

#include <vector>

class MyThread;
class MyThreadPool;
/// The class of idle thread container
/** It contains threads which is not in using */
class IdleThreadContainer
{

public:
    IdleThreadContainer();
    ~IdleThreadContainer();
    std::vector<MyThread*>::size_type size();
    void push(MyThread *m);
    void assign(int n,MyThreadPool* m);
    MyThread* top();
    void pop();
    void erase(MyThread *m);
private:
    std::vector<MyThread*> idle_thread_container_; /*!< The idle thread container itself*/
    typedef std::vector<MyThread*> Container;
    typedef Container::iterator Iterator;
};




#endif //UNTITLED_IDLETHREADCONTAINER_H
