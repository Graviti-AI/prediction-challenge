#pragma once
#include "core/simulator.h"

namespace core
{
class SimulatorImpl: public Simulator{
public:
    SimulatorImpl();
    virtual ~SimulatorImpl();

    // Simulator interface
public:
    void start();
    bool onUserState(Trajectory traj);
    Trajectory fetchEnv();
    void shutdown();
};
}
