//
// Created by lcr on 7/13/19.
//

#include "car.h"


state car::getstate(int time){
    for (auto record_state : car_state ){
        if (record_state.time_ == time){
            state state_at_time;
            state_at_time.time_ = time;
            state_at_time.length_ = record_state.length_;
            state_at_time.width_ = record_state.width_;
            state_at_time.psi_rad_ = record_state.psi_rad_;
            state_at_time.vx_ = record_state.vx_;
            state_at_time.vy_ = record_state.vy_;
            state_at_time.x_ = record_state.x_;
            state_at_time.y_ = record_state.y_;
            return state_at_time;
        }
    }
    state state_at_time;
    state_at_time.time_ = 0;
    state_at_time.length_ = 0;
    state_at_time.width_ = 0;
    state_at_time.psi_rad_ = 0;
    state_at_time.vx_ = 0;
    state_at_time.vy_ = 0;
    state_at_time.x_ = 0;
    state_at_time.y_ = 0;
    return state_at_time;

}