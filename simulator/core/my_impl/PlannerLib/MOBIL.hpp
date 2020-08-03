//
// Created by Chenran on 06/10/19.
//



#ifndef AGENTSIM_MOBIL_HPP
#define AGENTSIM_MOBIL_HPP


#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "IDM.hpp"


/**********************

  -----          -----         -----
 |  n  |        |     |       |  s  |
  -----          -----         -----
 ===========================================   Lane line
                  ^
                  |
  -----        -----          -----
 |  o  |      |  c  |        |  q  |
  -----        -----          -----

 **********************/

#define CHANGEING_THRESHOLD 0.1
#define POLITENESS_FACTOR 0.4

class MOBIL{

public:

    MOBIL(double current_states[4], double front_states[4], double back_states[4], double next_front_states[4], double next_back_states[4]);



    double lng_pos;
    double acceleration;
    double velocity;
    double desire_v;

    double q_lng_pos;
    double q_acceleration;
    double q_velocity;
    double q_desire_v;

    double o_lng_pos;
    double o_acceleration;
    double o_velocity;
    double o_desire_v;

    double s_lng_pos;
    double s_acceleration;
    double s_velocity;
    double s_desire_v;

    double n_lng_pos;
    double n_acceleration;
    double n_velocity;
    double n_desire_v;

    bool decision;


};







#endif //AGENTSIM_MOBIL_HPP