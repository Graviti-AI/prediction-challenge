//
// Created by m-1 on 10/24/18.
//

#include "IdleThreadContainer.hpp"
#include "MyThread.hpp"
#include <iostream>
#include <algorithm>
using namespace std;
IdleThreadContainer::IdleThreadContainer()
{
}


IdleThreadContainer::~IdleThreadContainer()
{
    int i = 0;
    for (Iterator it = idle_thread_container_.begin(); it != idle_thread_container_.end(); it++)
    {
        cout << i++ << endl;
        delete *it;
    }
}

///
/// Get the size of the idle thread container.
std::vector<MyThread*>::size_type IdleThreadContainer::size()
{
    return idle_thread_container_.size();
}

///
/// Push a thread into the idle thread container.
void IdleThreadContainer::push(MyThread *m)
{
    idle_thread_container_.push_back(m);
}

///
/// Get the last thread from the idle thread container.
void IdleThreadContainer::pop()
{
    idle_thread_container_.pop_back();
}

///
/// Erase a thread from the idle thread container.
void IdleThreadContainer::erase(MyThread *m)
{
    idle_thread_container_.erase(find(idle_thread_container_.begin(), idle_thread_container_.end(), m));
}

///
/// Assign a thread into the idle thread container of a thread pool.
void IdleThreadContainer::assign(int number, MyThreadPool *m)
{
    for (int i = 0; i < number; i++)
    {
        MyThread *p = new MyThread(m);
        idle_thread_container_.push_back(p);
    }
}

///
/// Get the last thread from the idle thread container.
MyThread* IdleThreadContainer::top()
{

    return idle_thread_container_.back();
}