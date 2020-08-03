//
// Created by m-1 on 10/24/18.
//

#include "MyThread.hpp"
#include "MyThreadPool.hpp"
#include <iostream>
using namespace std;
int MyThread::s_threadnumber = 0;
///
/// \param pool reference to the threadpool.
MyThread::MyThread(MyThreadPool *pool) :mythreadpool_(pool), isdetach_(true)
{
    s_threadnumber++;
    threadid_ = s_threadnumber;
}

///
/// Set thread to detach.
void MyThread::setisdetach(bool detach)
{
    isdetach_ = detach;
}

///
/// Assign a task to the thread.
void MyThread::Assign(Task *t)
{
    task_ =t;

}

///
/// Run method of thread.
void MyThread::Run()
{
    //cout <<"Thread:"<< threadid_ << " run ";
    task_->Run();
    mythreadpool_->RemoveThreadFromBusy(this);
}

///
/// Get id of a thread.
int MyThread::getthreadid()
{
    return threadid_;
}

///
/// Start a thread.
void MyThread::StartThread()
{
    thread_ = thread(&MyThread::Run, this);
    if (isdetach_ == true)
        thread_.detach();
    else
        thread_.join();
}

bool operator==(MyThread my1, MyThread my2)
{
    return my1.threadid_ == my2.threadid_;
}
bool operator!=(MyThread my1, MyThread my2)
{
    return !(my1.threadid_ == my2.threadid_);
}
