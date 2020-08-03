//
// Created by mscsim on 12/13/18.
//

#ifndef AGENTSIM_VIRTUALCAR_H
#define AGENTSIM_VIRTUALCAR_H

#include "Agent.hpp"
#include "../Behaviours/LaneletBehaviour.hpp"
#include "../Planners/VirtualCarPlanner.hpp"
#include "../Planners/AstarPlanner.hpp"
#include "../Planners/EBPlanner.hpp"
#include "../Planners/CILQRPlanner.hpp"
#include "../Controllers/BicyclePurePursuit.hpp"
#include "../Maps/MapInfo.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
using namespace lanelet;
///
/// VirtualCar is a kind of agent that is not exist in the real world.
/// Its state maybe comes from a .txt file or from a behaviour model.

class VirtualCar : public Agent {

public:
    VirtualCar(int id, Vector initialState);
    ~VirtualCar();
    explicit VirtualCar(int id, Vector initialState, LaneletBehaviour* behaviour, Planner *planner, Controller *controller, Model *model);

    void Run();
    AgentType getType() const override;
    void setBehaviour(LaneletBehaviour* b) { behaviour = b; };
    LaneletBehaviour* getBehaviour();

    bool CILQR_flag, first_run; // for CILQR
    MapInfo* testmapptr;
private:
    LaneletBehaviour* behaviour;
    AstarPlanner* planner;
    EBPlanner* ebplanner;
    CILQRPlanner* CILQR_planner;
    Bicycle_PurePursuit bicyclePurePursuiter;
    
    routing::LaneletPath shortestPath_;
    ConstLanelet startLanelet_;
    ConstLanelet destinationLanelet_;
    ConstLanelet currentLanelet_;

    void CILQRRun();
    Vector lane_change_s; // for CILQR
    int index_lane_change_s, last_left_lanelet_id, last_right_lanelet_id; // for CILQR
    double update_n, update_t_cost; // for CILQR
};


#endif //AGENTSIM_VIRTUALCAR_H
