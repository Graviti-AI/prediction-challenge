//
// Created by chenran on 05/01/20.
//

#ifndef AGENTSIM_EBPLANNER_H
#define AGENTSIM_EBPLANNER_H

#include "Planner.hpp"
#include <string>
#include <queue>
#include <vector>
#include "boost/multi_array.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_matching/LaneletMatching.h>
#include "../PlannerLib/EBplanner/EdgeAugumented_Planner.h"
#include "../PlannerLib/EBplanner/PurePursuit.h"
#include "../PlannerLib/EBplanner/NonconservativePlanner.h"
#include "../PlannerLib/EBplanner/Global_parameter.h"
#include "../PlannerLib/PurePursuit.h"
#include "../alglib/stdafx.h"
#include "../alglib/interpolation.h"
#include <bitset>
#include <time.h>
#include <array>
#include "../Agents/Agent.hpp"

using namespace std;
using namespace lanelet;
using namespace lanelet::matching;

struct onecar_ob{
    double x_;
    double y_;
    double length_;
    double width_;
    double theta_;
    double vx_;
    double vy_;
    double intersection_x_;
    double intersection_y_;
    double pass_possibility_;
    double t_in;
    double t_out;
    double s_in;
    double s_out;
};

struct car_ob_time{
    std::vector<onecar_ob> time_car;
};

struct CarObSet{
    std::vector<car_ob_time> car;
};



///
/// Planner using Astar and Optimization.
class EBPlanner : public Planner{
public:
    EdgeAugumented_Planner EBplannerA;
    EBPurePursuit purePursuitA;
    NonconservativePlanner nonconservative_planner;
    bool EB_path_planner_flag = false;
    CarObSet obstacles_set; //car obstacles set
    explicit EBPlanner(MapInfo* map);
    ~EBPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector update(Vector& currentState, alglib::spline1dinterpolant& ref_x, alglib::spline1dinterpolant& ref_y, const Vector &humanInput, std::vector<Agent*> agents, double s_now_);
    
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);
    
};



#endif //AGENTSIM_EBPLANNER_H
