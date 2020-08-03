//
// Created by m-1 on 10/24/18.
//

#ifndef UNTITLED_BUSYTHREADCONTAINER_H
#define UNTITLED_BUSYTHREADCONTAINER_H


#pragma once
#include <list>


class MyThread;
/// The class of busy thread container
/** It contains threads which is in using */
class BusyThreadContainer
{

public:
    BusyThreadContainer();
    ~BusyThreadContainer();
    void push(MyThread *m);
    std::list<MyThread*>::size_type size();
    void erase(MyThread *m);

private:
    std::list<MyThread*> busy_thread_container_; /*!< The busy thread container itself*/
    typedef std::list<MyThread*> Container;
    typedef Container::iterator Iterator;
};




#endif //UNTITLED_BUSYTHREADCONTAINER_H
