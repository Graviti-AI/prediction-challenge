//
// Created by lcr on 7/13/19.
//

#ifndef LANELET2_CAR_H
#define LANELET2_CAR_H

#include <vector>

class state{

public:
    int time_ = 0;
    double x_ = 0;
    double y_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double psi_rad_ = 0;
    double length_ = 0;
    double width_ = 0;
};
class car {

public:
    state getstate(int time);
    int id_;
    std::vector<state> car_state;
    double s_now;

};


#endif //LANELET2_CAR_H
