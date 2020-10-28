#pragma once
#include "trajectory.hpp"

namespace core
{

    struct SimulationScenario
    {
        SimulationScenario(const std::string &id, const std::string &name)
            : id(id), name(name)
        {
        }
        std::string id;
        std::string name;
    };

    class MySimulator
    {
    public:
        virtual ~MySimulator() {}
        virtual void start(const SimulationScenario &scenario, const std::string &config_file, const std::string &log_folder) = 0;
        virtual bool onPredictorState(std::vector<Trajectory> pred_trajs, std::vector<double> probability) = 0;
        virtual bool onPlannerState(Trajectory planned_traj) = 0;
        virtual SimulationEnv fetchEnv() = 0;
        virtual void shutdown() = 0;
    };

    MySimulator *create_simulator(int rviz_port);

} // namespace core
