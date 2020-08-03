#pragma once
#include "trajectory.hpp"

namespace core {

class MySimulator {
public:
    virtual ~MySimulator(){}
    virtual void start() = 0;
    virtual bool onUserState(Trajectory traj) = 0;
    virtual Trajectory fetchEnv() = 0;
    virtual void shutdown() = 0;
};

MySimulator* create_simulator();

}
