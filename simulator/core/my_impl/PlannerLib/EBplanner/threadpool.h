//
// Created by yuelin on 11/14/19.
//

#ifndef MULTITHREAD_THREADPOOL_H
#define MULTITHREAD_THREADPOOL_H

#include <condition_variable>
#include <functional>
#include <iostream>
#include <future>
#include <vector>
#include <thread>
#include <queue>

class ThreadPool
{
public:
    using Task = std::packaged_task<void()>;

    explicit ThreadPool(std::size_t numThreads)
    {
        start(numThreads);
    }

    ~ThreadPool()
    {
        stop();
    }

    void enqueue(Task task)
    {
        {
            std::unique_lock<std::mutex> lock{mEventMutex};
            mTasks.emplace(std::move(task));
        }
        mEventVar.notify_one();
    }

    bool queueEmpty()
    {
        return mTasks.empty();
    }

    int finished_task_number()
    {
        // cout<<"";
        // int result = count;
        return count;
    }

    void reste_task_number()
    {
        count = 0;
    }

    void add_task_number()
    {
        count += 1;
    }

private:
    std::vector<std::thread> mThreads;

    std::condition_variable mEventVar;

    std::mutex mEventMutex;
    bool mStopping = false;

    std::queue<Task> mTasks;

    std::mutex mCounterMutex;
    volatile int count = 0;

    void start(std::size_t numThreads)
    {
        //https://hackernoon.com/learn-c-multi-threading-in-5-minutes-8b881c92941f
        for (auto i = 0u; i < numThreads; ++i)
        {
            mThreads.emplace_back([=] {
                while (true)
                {
                    Task task;

                    {
                        std::unique_lock<std::mutex> lock{mEventMutex};
                        // no data, wait
                        mEventVar.wait(lock, [=] { return mStopping || !mTasks.empty(); });
                        // stop
                        if (mStopping && mTasks.empty())
                        {
                            break;
                        }

                        // get package
                        // std::cout << "-------taking task from queue-------" << std::endl;
                        task = std::move(mTasks.front());
                        mTasks.pop();
                        // std::cout << "-------number of tasks in queue: " << mTasks.size() << std::endl;

                        
                    }
                    // std::cout << "starting task" << std::endl;
                    // run package
                    task();
                    {
                        std::unique_lock<std::mutex> lock{mCounterMutex};
                        -- count;
                        // cout<<"";
                        // int ss = 0;
                        // for(int l = 0; l<100000000; l++){
                        //     ss = ss + l;
                        // }
                        // std::cout<<"------------"<< count<<endl;
                        // std::cout << "finished one task" << count <<std::endl;
                    }
                    
                }
            });
        }
    }

    void stop() noexcept
    {
        {
            std::unique_lock<std::mutex> lock{mEventMutex};
            mStopping = true;
        }

        mEventVar.notify_all();

        for (auto &thread : mThreads) {
            thread.join();
            std::cout << "thread killed" << std::endl;
        }
    }
};


#endif //MULTITHREAD_THREADPOOL_H
