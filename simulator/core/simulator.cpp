#include "simulator.h"
#include "core/impl/default_simulator_impl.h"

using namespace core;

Simulator *core::create_simulator()
{
    return new DefaultSimulatorImpl();
}
