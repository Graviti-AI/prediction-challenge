#pragma once
#include "core/simulator.hpp"

#include "Simulator/Simulator.hpp"


namespace core
{
class MySimulatorImpl: public MySimulator{
public:
    MySimulatorImpl(int rviz_port);
    ~MySimulatorImpl();

    // Simulator interface
public:
    void start();
    bool onUserState(Trajectory traj);
    Trajectory fetchEnv();
    void shutdown();

    Simulator simulator;
};
}
