

#ifndef AGENTSIM_NEWVIRTUALCAR_H
#define AGENTSIM_NEWVIRTUALCAR_H

#include "Agent.hpp"
#include "../Behaviours/FSM.hpp"
#include "../Planners/VirtualCarPlanner.hpp"
#include "../Planners/AstarPlanner.hpp"
#include "../Planners/CILQRPlanner.hpp"
#include "../Controllers/BicyclePurePursuit.hpp"
#include "math.h"
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
using namespace lanelet;

class NewVirtualCar : public Agent {

public:
    NewVirtualCar(int id, Vector initialState);
    ~NewVirtualCar();
    explicit NewVirtualCar(int id, Vector initialState, FSM* fsm, Planner *planner, Controller *controller, Model *model);
    Vector Intersection_pass(Vector& currentState, Vector &humanInput, std::vector<Agent*> agents);
    Vector Intersection_stop(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Cruise(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector IDM_following(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Merge(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    void Run();
    AgentType getType() const override;
    void setFSM(FSM* b) { fsm = b; };
    FSM* getFSM();
    double local_s;
    double last_s_;
    bool CILQR_flag, first_run;
    
private:
    FSM* fsm;
    AstarPlanner* planner;
    Bicycle_PurePursuit bicyclePurePursuiter;
    
    routing::LaneletPath shortestPath_;
    ConstLanelet startLanelet_;
    ConstLanelet destinationLanelet_;
    ConstLanelet currentLanelet_;
   
};


#endif //AGENTSIM_VIRTUALCAR_H
