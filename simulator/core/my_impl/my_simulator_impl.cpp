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

bool MySimulatorImpl::onUserState(Trajectory traj)
{
    assert(traj.size() == 30);
    uint64_t car_id = traj[0]->track_id;

    printf("\n====== Receive Traj from Client, Size = %d, Car ID = %d\n", (int)traj.size(), (int)car_id);

    for (auto state: traj){
        std::cout << "frame_id: " << state->frame_id << "; ";
        std::cout << "x: " << state->x << "; ";
        std::cout << "y: " << state->y << "; ";
        std::cout << std::endl;

        assert(state->track_id == car_id);
    }
    std::cout << std::endl;

    /*
    for(auto state: traj){
        std::cout<<"frame_id: "<<state->frame_id<<std::endl;
        std::cout<<"timestamp_ms: "<<state->timestamp_ms<<std::endl;
        std::cout<<"track_id: "<<state->track_id<<std::endl;
        std::cout<<"agent_type: "<<state->agent_type<<std::endl;
        std::cout<<"x: "<<state->x<<std::endl;
        std::cout<<"y: "<<state->y<<std::endl;
        std::cout<<"vx: "<<state->vx<<std::endl;
        std::cout<<"vy: "<<state->vy<<std::endl;
        std::cout<<"psi_rad: "<<state->psi_rad<<std::endl;
        std::cout<<"length: "<<state->length<<std::endl;
        std::cout<<"width: "<<state->width<<std::endl;
    }
    std::cout << "====================\n" << std::endl;
    */

    simulator.upload_traj(car_id, traj);
    return true;
}

core::Trajectory MySimulatorImpl::fetchEnv()
{
    Trajectory traj = simulator.fetch_history();
    assert(traj.size() == 10);
    
    return traj;
}

void MySimulatorImpl::shutdown()
{

}
