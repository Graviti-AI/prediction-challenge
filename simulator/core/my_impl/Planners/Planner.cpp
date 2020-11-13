

#include "Planner.hpp"

Planner::Planner(int dimState, int dimInput, MapInfo *map) {
    this->dimState = dimState;
    this->dimInput = dimInput;
    this->map = map;
    this->state = PlannerState::fine;
}

PlannerState Planner::get_state() {
    return state;
}

void Planner::set_state(PlannerState s) {
    state = s;
}
void set_traj(std::vector<TraPoints> traj) {
    this->planned_traj = traj;
}

std::vector<TraPoints> get_traj() {
    return planned_traj;
}