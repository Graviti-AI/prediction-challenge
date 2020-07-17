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
};

typedef std::vector<State*> Trajectory;
}
