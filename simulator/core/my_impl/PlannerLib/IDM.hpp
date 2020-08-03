//
// Created by Chenran on 06/10/19.
//



#ifndef AGENTSIM_IDM_HPP
#define AGENTSIM_IDM_HPP

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>



#define MAX_ACC 1.2
#define DESIRED_DECC -2
#define MIN_DISTANCE 8
#define TIME_GAP 0.01




class IDM{

public:
    IDM(double current_v, double lng_position, double desire_v, double front_lng_psition, double front_v);
    IDM(double current_v, double lng_position, double desire_v);


    std::vector<double> getstate(double t);

    double velocity_;
    double desired_velocity_;
    double lng_pos_;
    double front_velocity_;
    double front_lng_pos_;
    double acceleration;


};




#endif //AGENTSIM_IDM_HPP