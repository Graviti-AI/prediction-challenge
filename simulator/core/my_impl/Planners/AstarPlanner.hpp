//
// Created by chenran on 05/01/19.
//

#ifndef AGENTSIM_ASTARPLANNER_H
#define AGENTSIM_ASTARPLANNER_H

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
#include "../PlannerLib/AstarSolver.h"
#include "../PlannerLib/QuadProg++.hh"
#include "../PlannerLib/QPSetup.h"
#include "../PlannerLib/PurePursuit.h"
#include "../alglib/stdafx.h"
#include "../alglib/interpolation.h"
#include <time.h>
#include <array>
#include "../Agents/Agent.hpp"

using namespace std;
using namespace lanelet;
using namespace lanelet::matching;

///
/// Planner using Astar and Optimization.
class AstarPlanner : public Planner{
public:
    explicit AstarPlanner(Agent* agent_ibt, MapInfo* map);
    ~AstarPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info);

    //Vector update(Vector& currentState,  alglib::spline1dinterpolant& ref_x, alglib::spline1dinterpolant& ref_y, const Vector &humanInput, std::vector<Agent*> agents, double s);
private:
    double curvature[12500];
    double t_xx[5000], t_dx[5000], t_d2x[5000], t_yy[5000], t_dy[5000], t_d2y[5000];    //temp

    alglib::spline1dinterpolant last_spl_x, last_spl_y;
    int last_update_time;
};



#endif //AGENTSIM_ASTARPLANNER_H
