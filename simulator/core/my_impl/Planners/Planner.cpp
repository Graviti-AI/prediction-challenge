//
// SS 11/14/20
//

#include "Planner.hpp"

Planner::Planner(Agent* agent_ibt, int dimState, int dimInput, MapInfo *map) {
    this->agent_ibt_ = agent_ibt;
    this->dimState = dimState;
    this->dimInput = dimInput;
    this->map = map;
    this->state = SubprocessState::fine;
}

SubprocessState Planner::get_state() {
    return state;
}

void Planner::set_state(SubprocessState s) {
    state = s;
}

void Planner::set_traj(std::vector<TraPoints> traj) {
    this->planned_traj = traj;
}

std::vector<TraPoints> Planner::get_traj() {
    return planned_traj;
}