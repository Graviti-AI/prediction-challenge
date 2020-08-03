//
// Created by mscsim on 8/25/19.
//

#ifndef AGENTSIM_HUMANPEDESTRIANPLANNER_HPP
#define AGENTSIM_HUMANPEDESTRIANPLANNER_HPP

#include "../my_simulator_impl.hpp"
#include "Planner.hpp"

///
/// Planner for a human(VR) pedestrian
class HumanPedestrianPlanner : public Planner {
public:
    explicit HumanPedestrianPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
};

#endif //AGENTSIM_HUMANPEDESTRIANPLANNER_HPP