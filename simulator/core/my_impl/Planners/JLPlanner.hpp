//
// Created by Jinning on 10/22/19
//

#ifndef JINNING_PLANNER
#define JINNING_PLANNER

#include "Planner.hpp"
#include "../Behaviours/Behaviour.hpp"
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

using namespace lanelet;
using namespace lanelet::matching;

class JLPlanner : public Planner {
public:
    explicit JLPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);

};


#endif //JINNING_PLANNER
