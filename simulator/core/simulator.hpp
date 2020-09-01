#pragma once
#include "trajectory.hpp"

namespace core {

class MySimulator {
public:
    virtual ~MySimulator(){}
    virtual void start() = 0;
    virtual bool onUserState(std::vector<Trajectory> pred_trajs, std::vector<double> probability) = 0;
    virtual SimulationEnv fetchEnv() = 0;
    virtual void shutdown() = 0;
};

MySimulator* create_simulator(int rviz_port);

}
