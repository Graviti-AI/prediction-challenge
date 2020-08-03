//
// Created by Fan on 2018/9/26.
//

#ifndef AGENTSIM_TRIVIALPLANNER_H
#define AGENTSIM_TRIVIALPLANNER_H

#include "Planner.hpp"
///
/// Planner for the real car
class TrivialPlanner : public Planner {
public:
    explicit TrivialPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
};


#endif //AGENTSIM_TRIVIALPLANNER_H
