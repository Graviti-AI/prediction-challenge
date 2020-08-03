//
// Created by Chenran on 06/10/19.
//



#ifndef AGENTSIM_OPTIMALTIME_HPP
#define AGENTSIM_OPTIMALTIME_HPP

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>


class OptimalTime{

public:

    OptimalTime(double v, double a, double lng_position, double ds, double d_v, double wA, double wT):
            velocity_(v), desired_velocity_(d_v), acceleration_(a), lng_pos_(lng_position), wA_(wA), wT_(wT), dest_dis_(ds),c1(0),c2(0),c3(0),c4(0),c5(0),optimal_T(0)
    {}
    void ForStop();
    void FreeFlow();
    std::vector<double> getstate(double t);


    double lng_pos_;

// private:

    double velocity_;
    double desired_velocity_;
    double acceleration_;

    double wT_;
    double wA_;
    double dest_dis_;    // Also noted as sf


    double c1;
    double c2;
    double c3;
    double c4;
    double c5;

    double optimal_T;
};













#endif //AGENTSIM_OPTIMALTIME_HPP



