#pragma once

#include "../trajectory.hpp"


enum SimulatorState {
    Running = 0,
    Paused = 1,
    Reset = 2
};

const double SIM_TICK = 0.01; // simulation step (in seconds).