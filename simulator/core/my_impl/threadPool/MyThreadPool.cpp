//
// Created by m-1 on 10/24/18.
//

#include "MyThreadPool.hpp"
#include <iostream>
using namespace std;
///
/// \param number reference to the number of threads in the thread pool.
MyThreadPool::MyThreadPool(int number)
{
    issurvive_ = true;
    number_of_thread_ = number;
    idle_thread_container_.assign(number, this);
    thread_this_ =thread(&MyThreadPool::Start, this);
    thread_this_.detach();
}
MyThreadPool::~MyThreadPool()
{

}

///
/// End the thread pool.
void MyThreadPool::EndMyThreadPool()
{
    issurvive_ =false;
}

///
/// Add an idle thread to the thread pool.
void  MyThreadPool::AddIdleThread(int n)
{
    idle_mutex_.lock();
    idle_thread_container_.assign(n, this);
    number_of_thread_ += n;
    idle_mutex_.unlock();
}

///
/// Start the thread pool.
/// It will keep scan whether exists idle threads in idle_thread_container_
/// and invoke idle threads to handle tasks and put that thread into busy_thread_container_.
void MyThreadPool::Start()
{

    while (true)
    {
        if (issurvive_==false)
        {
            busy_mutex_.lock();
            if (busy_thread_container_.size()!=0)
            {
                busy_mutex_.unlock();
                continue;
            }
            busy_mutex_.unlock();
            break;
        }
        idle_mutex_.lock();
        if (idle_thread_container_.size() == 0)
        {
            idle_mutex_.unlock();
            continue;
        }
        idle_mutex_.unlock();
        task_mutex_.lock();
        if (task_container_.size() == 0)
        {
            task_mutex_.unlock();
            continue;
        }
        Task *b = task_container_.top();;
        task_container_.pop();
        task_mutex_.unlock();

        idle_mutex_.lock();
        MyThread *mythread = idle_thread_container_.top();;
        idle_thread_container_.pop();
        mythread->Assign(b);
        idle_mutex_.unlock();

        busy_mutex_.lock();
        busy_thread_container_.push(mythread);
        busy_mutex_.unlock();
        mythread->StartThread();
    }
}

///
/// Add a task to the thread pool.
void MyThreadPool::AddTask(Task *Task, int priority = (PRIORITY::NORMAL))
{

    Task->SetPriority(priority);
    task_mutex_.lock();
    task_container_.push(Task);
    task_mutex_.unlock();
}

///
/// Remove a thread from busy thread container.
void MyThreadPool::RemoveThreadFromBusy(MyThread *myThread)
{

    busy_mutex_.lock();
    //cout << "Thread:" << myThread->getthreadid()<< " remove from busylist" << endl;
    busy_thread_container_.erase(myThread);
    busy_mutex_.unlock();

    idle_mutex_.lock();
    idle_thread_container_.push(myThread);
    idle_mutex_.unlock();
}
