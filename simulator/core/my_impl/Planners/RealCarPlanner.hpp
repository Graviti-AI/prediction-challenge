//
// Created by msclab on 10/31/18.
//

#ifndef AGENTSIM_REALCARPLANNER_H
#define AGENTSIM_REALCARPLANNER_H


#include "Planner.hpp"

///
/// Planner for the real car
class RealCarPlanner : public Planner {
public:
    explicit RealCarPlanner();
    void updatepre(PlannerPre& new_pre);
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
};


#endif //AGENTSIM_REALCARPLANNER_H
