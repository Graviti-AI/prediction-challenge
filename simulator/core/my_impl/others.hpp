#pragma once


enum SimulatorState {
    Running = 0,
    Paused = 1,
    Reset = 2
};

const double SIM_TICK = 0.5; // simulation step (in seconds).