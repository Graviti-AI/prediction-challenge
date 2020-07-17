#include "simulator.h"
#include "core/impl/simulator_impl.h"

using namespace core;

Simulator *core::create_simulator()
{
    return new SimulatorImpl();
}
