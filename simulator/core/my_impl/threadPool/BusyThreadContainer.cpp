//
// Created by m-1 on 10/24/18.
//

#include "BusyThreadContainer.hpp"
#include "MyThread.hpp"
#include <iostream>
#include <algorithm>
using namespace std;
BusyThreadContainer::BusyThreadContainer()
{
}
BusyThreadContainer::~BusyThreadContainer()
{
}

///
/// Push a thread into the busy thread container.
void BusyThreadContainer::push(MyThread *m)
{
    busy_thread_container_.push_back(m);
}

///
/// Erase a thread from the busy thread container.
void  BusyThreadContainer::erase(MyThread *m)
{
    busy_thread_container_.erase(find(busy_thread_container_.begin(),busy_thread_container_.end(),m));
}

///
/// Get the size of the busy thread container.
std::list<MyThread*>::size_type BusyThreadContainer::size()
{
    return busy_thread_container_.size();
}

