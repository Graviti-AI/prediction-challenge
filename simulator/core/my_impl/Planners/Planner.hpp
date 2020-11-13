//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_PLANNER_HPP
#define AGENTSIM_PLANNER_HPP

#include <vector>
#include <stdexcept>
typedef std::vector<double> Vector;
class Agent;
class MapInfo;
struct Obstacle_info;
enum PlannerType {
    Astar = 0,
    EB,
    CILQR
};
struct AstarPre{
    float w_a=0;
    float w_an=0;
    float w_v=1;
};
struct EBPre{

};

struct CILQRPre{

};
struct PlannerPre
{
    AstarPre AstarPre_;
    EBPre EBPre_;
    CILQRPre CILQRPre_;
};

///
/// Parent class for all planners
class Planner{
public:
    explicit Planner(Agent* agent_ibt, int dimState, int dimInput, MapInfo *map = nullptr);

    virtual Vector update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents, std::vector<Obstacle_info> obstacle_info) = 0;
    virtual void updatepre(PlannerPre& new_pre) = 0;
    bool false_flag = false;
    PlannerState get_state();
    void set_state(PlannerState s);

protected:
    const int dimState; /*!< dimension of the state vector*/
    const int dimInput; /*!< dimension of the input vector.*/
    MapInfo *map;
<<<<<<< HEAD
    PlannerState state;
=======
    Agent* agent_ibt_;

>>>>>>> orz
};


#endif //AGENTSIM_PLANNER_HPP
