#include "simulator_impl.h"
using namespace core;

SimulatorImpl::SimulatorImpl()
{

}

SimulatorImpl::~SimulatorImpl()
{

}

void SimulatorImpl::start()
{

}

bool SimulatorImpl::onUserState(Trajectory /*traj*/)
{
    return true;
}

Trajectory SimulatorImpl::fetchEnv()
{
    return Trajectory();
}

void SimulatorImpl::shutdown()
{

}
