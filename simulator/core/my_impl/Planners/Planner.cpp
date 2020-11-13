

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