#include <iostream>
#include <assert.h>
#include "my_simulator_impl.hpp"

#include "Simulator/Simulator.hpp"
#include "threadPool/MyThreadPool.hpp"
#include "Controllers/PedestrianController.hpp"
#include "Models/PedestrianModel.hpp"
#include "Planners/PedestrianPlanner.hpp"
#include "Planners/HumanPedestrianPlanner.hpp"


using namespace core;



MySimulatorImpl::MySimulatorImpl(int rviz_port):simulator(rviz_port)
{
    
}

MySimulatorImpl::~MySimulatorImpl()
{

}

void MySimulatorImpl::start()
{
    std::cout << "Simulator Runs" << std::endl;
    simulator.run();
}

bool MySimulatorImpl::onUserState(std::vector<Trajectory> pred_trajs, std::vector<double> probability)
{
    assert(pred_trajs.size() == 30);
    assert(probability.size() == 30);

    //TODO: update user state
    assert(false);
    return true;
}

core::SimulationEnv MySimulatorImpl::fetchEnv()
{
    Trajectory traj = simulator.fetch_history();
    assert(traj.size() == 10);

    //TODO: create SimulationEnv
    assert(false);
    core::SimulationEnv env;
    env.myTraj = traj;
    env.map_name = "todo.map";
    env.other_trajs = std::vector<Trajectory>();

    return env;
}

void MySimulatorImpl::shutdown()
{

}
