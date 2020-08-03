#pragma once
#include "core/simulator.h"

namespace core
{
class DefaultSimulatorImpl: public Simulator{
public:
    DefaultSimulatorImpl();
    virtual ~DefaultSimulatorImpl();

    // Simulator interface
public:
    void start();
    bool onUserState(Trajectory traj);
    Trajectory fetchEnv();
    void shutdown();
};
}
