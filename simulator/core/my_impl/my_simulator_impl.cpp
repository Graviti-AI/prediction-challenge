#include <iostream>
#include "my_simulator_impl.hpp"

#include "Simulator/Simulator.hpp"
#include "threadPool/MyThreadPool.hpp"
#include "Controllers/PedestrianController.hpp"
#include "Models/PedestrianModel.hpp"
#include "Planners/PedestrianPlanner.hpp"
#include "Planners/HumanPedestrianPlanner.hpp"


using namespace core;



MySimulatorImpl::MySimulatorImpl()
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

bool MySimulatorImpl::onUserState(Trajectory traj)
{
    std::cout << "\n====== Receive User State from Client, Size = " << traj.size() << std::endl;
    for(auto state: traj){
        std::cout<<"frame_id: "<<state->frame_id<<std::endl;
        std::cout<<"timestamp_ms: "<<state->timestamp_ms<<std::endl;
        std::cout<<"agent_type: "<<state->agent_type<<std::endl;
        std::cout<<"x: "<<state->x<<std::endl;
        std::cout<<"y: "<<state->y<<std::endl;
        std::cout<<"vx: "<<state->vx<<std::endl;
        std::cout<<"track_id: "<<state->track_id<<std::endl;
        std::cout<<"vy: "<<state->vy<<std::endl;
        std::cout<<"psi_rad: "<<state->psi_rad<<std::endl;
        std::cout<<"length: "<<state->length<<std::endl;
        std::cout<<"width: "<<state->width<<std::endl;
    }
    std::cout << "====================\n" << std::endl;

    simulator.upload_traj(0, traj);
    return true;
}

core::Trajectory MySimulatorImpl::fetchEnv()
{
    Trajectory traj = simulator.randomly_sample(0);    //TODO:
    return traj;
}

void MySimulatorImpl::shutdown()
{

}
