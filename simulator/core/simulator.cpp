#include "simulator.hpp"
#include "core/my_impl/my_simulator_impl.hpp"

using namespace core;

MySimulator *core::create_simulator(int rviz_port)
{
    return new MySimulatorImpl(rviz_port);
}
