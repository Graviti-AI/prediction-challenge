// PyPlanner.cpp
// SS 11/6/18

#include "PyPlanner.hpp"

PyPlanner::PyPlanner(Agent* agent_ibt, MapInfo* map) 
: Planner(agent_ibt, 6, 5 * PLAN_STEP,map) {;}

std::vector<TraPoints> PyPlanner::update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info)
{
    assert(state == SubprocessState::fine);
    state = SubprocessState::wait4fetch;

    while (state != SubprocessState::wait4update){
        // Ask Yaofeng - is this where I need to call it!?
        usleep(1e6 * SIM_TICK); 
    }
    return planned_traj;

}

void updatepre(PlannerPre& new_pre)
{
  ;
}