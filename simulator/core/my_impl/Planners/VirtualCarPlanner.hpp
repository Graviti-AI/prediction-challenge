//
// Created by mscsim on 12/13/18.
//

#ifndef AGENTSIM_VIRTUALCARPLANNER_H
#define AGENTSIM_VIRTUALCARPLANNER_H

#include "Planner.hpp"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Primitive.h>
///
/// Planner for the virtual car
class VirtualCarPlanner : public Planner{
public:
    explicit VirtualCarPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
    Vector update(Vector& currentState, Vector& targetState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector update(Vector& currentState, std::vector<std::pair<double, double>>& trajectory, const Vector &humanInput, std::vector<Agent*> agents);

    int tmpCount = 0;
};


#endif //AGENTSIM_VIRTUALCARPLANNER_H
