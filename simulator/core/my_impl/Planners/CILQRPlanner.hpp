#ifndef AGENTSIM_CILQRPLANNER_H
#define AGENTSIM_CILQRPLANNER_H

#include <Eigen/Core>
#include "Planner.hpp"
#include "../PlannerLib/CILQRTypeDef.h"
#include "../Agents/Agent.hpp"
class LaneletBehaviour;
class CAStarSolver;
class CILQRSolver;
class MapInfo;

/// Planner using CILQR and Optimization.
class CILQRPlanner : public Planner{
public:
    explicit CILQRPlanner(Agent *agent_ibt, MapInfo* map);
    ~CILQRPlanner();
    void updatepre(PlannerPre& new_pre){};
    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent *> agents, std::vector<Obstacle_info> obstacle_info);
    
    Vector getResult();
    void setId(int id_) { id = id_;}
    std::tuple<double, double, double, double, int, int> toArcCoordinatesBS(const LineString2d& path, const Vector& each_len, double px, double py, int index = -1);

    void getRoutingReferencePath(double & total_length);
    Vector solve(Vector& currentState, const Vector &humanInput, std::vector<Agent *> &agents, std::vector<Obstacle_info> & obstacle_info);
    void setObsPrediction(int agent_id, std::vector< std::vector<double> > & future_state, double half_L, double half_W, double max_acc, double min_acc, CILQRSolver *p);

private:
    Eigen::MatrixXd last_output_x, last_output_u;
    CILQRParameter CILQR_param;
    CAStarParameter AStar_param;
    CAStarSolver* AStar_solver;
    MapInfo* mapinfo;

    int id; // Car id
    int times, plan_times; // denotes we will make a plan every (plan_times*CILQR_Ts) seconds
    bool first_initial, first_run; // for the first run, there is no last_output
    alglib::spline1dinterpolant route_x, route_y; // route_path
    double length_route_path, route_start_s, dest_x, dest_y;
    LineString2d route_path;
    Vector route_path_each_len;
};


#endif //AGENTSIM_CILQRPLANNER_H