#pragma once
#include "trajectory.h"

namespace core {

class Simulator {
public:
    virtual ~Simulator(){}
    virtual void start() = 0;
    virtual bool onUserState(Trajectory traj) = 0;
    virtual Trajectory fetchEnv() = 0;
    virtual void shutdown() = 0;
};

Simulator* create_simulator();

}
