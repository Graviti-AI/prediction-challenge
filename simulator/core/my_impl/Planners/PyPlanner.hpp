#ifndef AGENTSIM_PYPLANNER_H
#define AGENTSIM_PYPLANNER_H

#include "Planner.hpp"
#include "../simulator_state.hpp"
#include <vector>

const int PLAN_STEP = 500;
///
/// C++ uses this to work with Planner written in Python
///

class PyPlanner : public Planner {
public:
    explicit PyPlanner(Agent* agent_ibt, MapInfo* map);
    ~PyPlanner();
    void updatepre(PlannerPre& new_pre);
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);

    // Vector last_humanInput; // no longer used, ignore
    std::vector<Obstacle_info> last_obstacle_info;

};


#endif //AGENTSIM_PYPLANNER_H
