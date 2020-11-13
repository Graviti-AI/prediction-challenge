#pragma once
#include "core/simulator.hpp"

#include "Simulator/Simulator.hpp"

namespace core
{
    class MySimulatorImpl : public MySimulator
    {
    public:
        MySimulatorImpl(int rviz_port);
        ~MySimulatorImpl();

        // Simulator interface
    public:
        void start(const SimulationScenario& scenario, const std::string &config_file, const std::string &log_folder, const bool verbose);
        bool onUserState(std::vector<Trajectory> pred_trajs, std::vector<double> probability);
        SimulationEnv fetchEnv();
        void shutdown();

        Simulator simulator;
    };
} // namespace core
