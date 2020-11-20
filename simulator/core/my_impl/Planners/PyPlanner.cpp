// PyPlanner.cpp
// SS 11/6/18

#include "PyPlanner.hpp"

PyPlanner::PyPlanner(Agent* agent_ibt, MapInfo* map) 
: Planner(agent_ibt, 6, 5 * PLAN_STEP, map) {;}

Vector PyPlanner::update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info)
{
    assert(state == SubprocessState::fine);
    state = SubprocessState::wait4fetch;

    while (state != SubprocessState::wait4update){
        usleep(1e6 * SIM_TICK); 
    }

    std::vector<double> ret = std::vector<double>(5 * PLAN_STEP);
    for (int i = 0; i < PLAN_STEP; i++) {
        ret[5*i + 0] = planned_traj[i].t; // time step
        ret[5*i + 0] = planned_traj[i].x; // x position
        ret[5*i + 0] = planned_traj[i].y; // y position
        ret[5*i + 0] = planned_traj[i].v; // velocity
        ret[5*i + 0] = planned_traj[i].theta; // angle
    }
    return ret;
}

void PyPlanner::updatepre(PlannerPre& new_pre)
{
  ;
}