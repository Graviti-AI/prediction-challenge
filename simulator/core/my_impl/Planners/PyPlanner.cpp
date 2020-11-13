// PyPlanner.cpp
// SS 11/6/18

#include "PyPlanner.hpp"

PyPlanner::PyPlanner(Agent* agent_ibt, MapInfo* map) 
: Planner(agent_ibt, 6, 5 * PLAN_STEP,map) {;}

Vector PyPlanner::update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info)
{
    assert(state == PlannerState::fine);
    state = PlannerState::wait4fetch;

    while (state != PlannerState::wait4update){
        usleep(1e6 * SIM_TICK); 
    }
    return PlannedTraj;
}

void updatepre(PlannerPre& new_pre)
{
  ;
}