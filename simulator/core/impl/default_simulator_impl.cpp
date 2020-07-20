#include <iostream>
#include "default_simulator_impl.h"
using namespace core;

DefaultSimulatorImpl::DefaultSimulatorImpl()
{

}

DefaultSimulatorImpl::~DefaultSimulatorImpl()
{

}

void DefaultSimulatorImpl::start()
{

}

bool DefaultSimulatorImpl::onUserState(Trajectory traj)
{
    std::cout<<"========"<<std::endl<<"get user state, size: "<<traj.size()<<std::endl;
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
    std::cout<<std::endl;

    return true;
}

Trajectory DefaultSimulatorImpl::fetchEnv()
{
    auto state = new State();
    state->track_id=1;
    state->frame_id=2;
    state->timestamp_ms=3;
    state->agent_type="car";
    state->x=4;
    state->y=5;
    state->vx=6;
    state->vy=7;
    state->psi_rad=8;
    state->length=9;
    state->width=0;

    Trajectory traj;
    traj.emplace_back(state);
    return traj;
}

void DefaultSimulatorImpl::shutdown()
{

}
