//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_HUMANCARPLANNER_HPP
#define AGENTSIM_HUMANCARPLANNER_HPP

#include "Planner.hpp"

///
/// Planner for a human car, which passes human inputs directly
class HumanCarPlanner : public Planner {
public:
    explicit HumanCarPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
};


#endif //AGENTSIM_HUMANCARPLANNER_HPP
