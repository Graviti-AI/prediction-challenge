#ifndef AGENTSIM_PYPLANNER_H
#define AGENTSIM_PYPLANNER_H

#include "Planner.hpp"
#include <vector>

///
/// C++ uses this to work with Planner written in Python
///

class PyPlanner : public Planner {
public:
    explicit PyPlanner();
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
};


#endif //AGENTSIM_PYPLANNER_H
