//
// Created by mscsim on 8/13/19.
//

#ifndef AGENTSIM_PEDESTRIANPLANNER_HPP
#define AGENTSIM_PEDESTRIANPLANNER_HPP


#include "../my_simulator_impl.hpp"
#include "Planner.hpp"

///
/// Planner for a pedestrian
class PedestrianPlanner : public Planner {
public:
    explicit PedestrianPlanner();
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
};

#endif //AGENTSIM_PEDESTRIANPLANNER_HPP
