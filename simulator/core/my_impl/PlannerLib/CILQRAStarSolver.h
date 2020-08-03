#ifndef CILQR_ASTAR_SOLVER_H
#define CILQR_ASTAR_SOLVER_H

#include <vector>
#include <Eigen/Core>
#include <boost/multi_array.hpp>
#include "CILQRMinHeap.h"
#include "CILQRTypeDef.h"

#define Max_Capacity_S 1000
// road length 100m * 10(resolution 0.1m)
#define Max_Capacity_V 350
// max v 35m/s * 10(resolution 0.1m)
#define Max_Capacity_N 80
// max N 80
#define Max_Related_Lane 3
// Now, the algorithm only considers 3 lanes.

class CAStarSolver
{
public:
    CAStarSolver(CAStarParameter param_, int max_s_for_list_, int max_v_for_list_);
    ~CAStarSolver();

    CNode start_node;
    int num_lane, max_s_for_lane; // max_s_for_lane is for lane_s, lane_curv, lane_theta, lane_width, refer_lane
    bool right_lane_exist, left_lane_exist;
    Eigen::MatrixXd init_seq_u, init_seq_x; // init seq, make sure ego car is at the centerline of one lane
    Eigen::MatrixXd lane_curv, lane_s, lane_width, lane_theta; // lane info
    Eigen::VectorXi refer_lane; // 0: left lane, 1: center lane, 2: right lane; Center lane refers to the lane of inital position 

    bool run();
    void addObstacle(const ObstacleParameter& obs_);
    Eigen::MatrixXd getOutputControl();
    int getIndex(const double & s, const int & lane);
    
    void setParam(CAStarParameter param_);
    void initSetup();

private:
    CAStarParameter param; // param of problem
    CMinHeap open_list; // open list
    int max_s_for_list, max_v_for_list;
    boost::multi_array<float, 4> cost_list; // for open list with duplicated state, store the mim cost
    boost::multi_array<int, 4> check_list; // 0 new visited, 1 open list, 2 closed list
    std::vector<ObstacleParameter> obstacles; // obstacles
    Eigen::MatrixXd output_u;
    int consider_vel; // 1: consider vel in constraints; 0: not consider
    std::vector<std::tuple<int, int, int, int>> rec;

    bool hasCollisionLK(int lane, int tk0, double s0, double s, double v);
    bool hasCollisionLC(int cur_lane, int tar_lane, int tk0, double s0, double v0, int T1, int T2,
                            double a1, double a2, double alpha, double R);
    bool isLinearRoad(int cur_lane, double s0, double s1);
    double sign(const double & a);
    std::vector<CNode> expandLeafNode(const CNode& node);
    void setOutput(const CNode& node);
};

#endif