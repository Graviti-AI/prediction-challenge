//
// Created by LCR on 7/15/20.
//

#ifndef AOBEHAVIOUR_HPP
#define AOBEHAVIOUR_HPP

#include "Behaviour.hpp"

class AoBehaviour : public Behaviour {

public:
    AoBehaviour(Agent* agent_ibt, BehaviourType t);
    Vector update(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Following(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Merging(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    Vector Stopping(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    void getObstacle(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents);
    
    Vector get_stop_plan(double remaining, double v_init, double a_init);
    // return: Vector{a1, b1, c1, a2, b2, c2, d2, t_stop};

    Vector interpolate_stop(Vector plan, double t_new);
    // return: Vector{s_new, v_new};
    
    double last_time;
    std::vector<std::pair<int, int>> allWayStopWaitingidList;

private:
    double t_xx[500], t_dx[500], t_d2x[500], t_yy[500], t_dy[500], t_d2y[500];    //temporal variables
    int last_stop_update;
    int last_stop_t0;
    double last_stop_s;
    Vector last_stop_plan; 
};




#endif //AOBEHAVIOUR_HPP
