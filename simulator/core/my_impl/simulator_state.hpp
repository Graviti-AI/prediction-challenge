#pragma once

#include "../trajectory.hpp"


enum SimulatorState {
    Running = 0,
    Paused = 1,
    Reset = 2
};

const int SIM_TICK_MS = 100;                        // simulation step (in ms).
const double SIM_TICK_SECOND = 0.001 * SIM_TICK_MS;      // simulation step (in seconds).

const int REPLAY_INTERVAL = 100 / SIM_TICK_MS;      // NOTE: SIM_TICK_MS must be a factor of 100