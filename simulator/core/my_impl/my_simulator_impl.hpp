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
    void start(const std::string& config_file);
    bool onUserState(std::vector<Trajectory> pred_trajs, std::vector<double> probability);
    SimulationEnv fetchEnv();
    void shutdown();

    Simulator simulator;
};
}
