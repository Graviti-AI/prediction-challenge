#pragma once

#include "../trajectory.hpp"
#include "Maps/MapInfo.hpp"


enum SimulatorState {
    Running = 0,
    Paused = 1,
    Reset = 2
};

enum SubprocessState {
    fine = 0,

    // designed for PyPredictor and PyPlanner
    wait4fetch = 1,    
    wait4upload = 2,
    wait4update = 3,
};

struct TraPoints
{
    double t;
    double x;
    double y;
    double theta;
    double delta_theta;
    double v;
    double a;
    double jerk;
    ConstLanelet current_lanelet;
    double s_of_current_lanelet;
    double d_of_current_lanelet;
};

const double SIM_TICK = 0.01; // simulation step (in seconds).