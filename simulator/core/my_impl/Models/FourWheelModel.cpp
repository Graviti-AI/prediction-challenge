//
// Created by mscsim on 8/29/18.
//

#include <iostream>
#include "FourWheelModel.hpp"

#include "FourWheelModel/Car_4wheel.h"

/// Constructor.
/// Dimension of state vector is 6 (x, y, yaw, speed x, speed y, speed yaw)
/// Dimension of intermediate vector is 3 (proportional to gas pedal, brake pedal, and steering angle)
FourWheelModel::FourWheelModel() : Model(6, 3) {
    this->innerModel = new Car_4wheel();
    this->count = 0;
}

/// Wrapper for the complex model
/// \param state Dimension of state vector is 6 (x, y, yaw, speed x, speed y, speed yaw)
/// \param intermediate Dimension of intermediate vector is 3 (proportional to gas pedal, brake pedal, and steering angle)
/// \return the state vector of this car for the next iteration
Vector FourWheelModel::update(Vector state, Vector intermediate) {
    std::clock_t current_time = std::clock();
    double time_gap = (current_time - last_time) / (double) CLOCKS_PER_SEC;
    last_time = current_time;
    if (this->count == 0) {
        this->innerModel->Carinit(state[0], state[1], state[3], state[4], state[2], state[5]);
    } else {
        this->innerModel->getinput(intermediate[0], intermediate[1], intermediate[2]);
        this->innerModel->Carupdate(time_gap);
    }

    this->count++;


    return Vector{
            100 * this->innerModel->output[0],
            100 * this->innerModel->output[1],
            this->innerModel->output[2],
            100 * this->innerModel->output[3],
            100 * this->innerModel->output[4],
            this->innerModel->output[5]
    };

}

FourWheelModel::~FourWheelModel() {
    delete this->innerModel;
}
