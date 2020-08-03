#ifndef AGENTSIM_CILQRPLANNER_H
#define AGENTSIM_CILQRPLANNER_H

#include <Eigen/Core>
#include "Planner.hpp"
#include "../PlannerLib/CILQRTypeDef.h"
#include "../Agents/Agent.hpp"
class LaneletBehaviour;
class CAStarSolver;

/// Planner using CILQR and Optimization.
class CILQRPlanner : public Planner{
public:
    explicit CILQRPlanner();
    ~CILQRPlanner();
    void updatepre(PlannerPre& new_pre){};

    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector update(Vector& currentState, LineString2d& traj, const Vector &humanInput, std::vector<Agent*> agents);

    Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info){};
    
    Vector getResult();
    void setId(int id_) {
        id = id_;
    }
    std::tuple<double, double, double, double, int, int> toArcCoordinatesBS(const LineString2d& path, const Vector& each_len, double px, double py, int index = -1);

    const LaneletBehaviour* behaviour;
    
    bool enable_prediction;
    int getN() { return CILQR_param.N; }
    void setObsPrediction(int agent_id, std::vector< std::vector<double> > & future_state, double half_L, double half_W, double max_acc, double min_acc);
    std::vector< std::pair<int, ObstacleParameter> > obs_deter; // enable_prediction = true; provided by the prediction module

private:
    Eigen::MatrixXd last_output_x, last_output_u;
    CILQRParameter CILQR_param;
    CAStarParameter AStar_param;
    CAStarSolver* AStar_solver;

    int times, plan_times; // denotes we will make a plan every (plan_times*CILQR_Ts) seconds
    bool first_run; // for the first run, there is no last_output
    int id; // Current Lanelet id
};


#endif //AGENTSIM_CILQRPLANNER_H