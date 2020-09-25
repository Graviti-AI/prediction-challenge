#pragma once
#include <string>
#include <vector>

namespace core {

struct State {
    uint64_t track_id;
    uint64_t frame_id;
    uint64_t timestamp_ms;
    std::string agent_type;
    double x;
    double y;
    double vx;
    double vy;
    double psi_rad;
    double length;
    double width;
    double jerk;
    uint64_t current_lanelet_id;
    double s_of_current_lanelet;
    double d_of_current_lanelet;
};

typedef std::vector<State*> Trajectory;

struct SimulationEnv {
    Trajectory my_traj;
    std::string map_name;
    std::vector<Trajectory> other_trajs;

    bool paused;    // whether the simulator has paused.
};

}
