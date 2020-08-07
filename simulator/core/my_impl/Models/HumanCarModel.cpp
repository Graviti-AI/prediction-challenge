//
// Created by mscsim on 8/23/18.
//

#include "HumanCarModel.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>

#include "../simulator_state.hpp"

/// Constructor.
/// Dimension of state vector is 6 (x, y, yaw, speed x, speed y, speed yaw)
/// Dimension of intermediate vector is 3 (proportional to gas pedal, brake pedal, and steering angle)
HumanCarModel::HumanCarModel() : Model(6, 3) {

}

/// Implements the simple kinematic model
/// \param state Dimension of state vector is 6 (x, y, yaw, speed x, speed y, speed yaw)
/// \param intermediate Dimension of intermediate vector is 3 (proportional to gas pedal, brake pedal, and steering angle)
/// \return the state vector of this car for the next iteration
Vector HumanCarModel::update(Vector state, Vector intermediate) {

    if (state.size() != this->dimState) {
        throw std::runtime_error("bad dimension");
    }

    if (intermediate.size() != this->dimIntermediate) {
        throw std::runtime_error("bad dimension");
    }

    double locationX = state[0];
    double locationY = state[1];
    double locationYaw = state[2];
    double speedX = state[3];
    double speedY = state[4];
    double speedYaw = state[5];

    std::clock_t current_time = std::clock();
    double time_gap = (current_time - last_time) / (double) CLOCKS_PER_SEC;
    std::cout << "time_gap : " << time_gap << std::endl;
    last_time = current_time;
    locationX += (time_gap * speedX);
    locationY += (time_gap * speedY);
    locationYaw += (time_gap * speedYaw);

    double gasSignal = intermediate[0];
    double brakeSignal = intermediate[1];
    double steeringSignal = intermediate[2];

    double acc = gasSignal - brakeSignal;

    double speed = sqrt(speedX * speedX + speedY * speedY);

    speedX = (speed + time_gap * acc) * cos(locationYaw);
    speedY = (speed + time_gap * acc) * sin(locationYaw);

    const double length = 4.0;
    speedYaw = (speed * tan(0.001 * steeringSignal) / length);

    const double DAMP = 0.99;

    speedX *= (1.0 - (1.0 - DAMP) * time_gap);
    speedY *= (1.0 - (1.0 - DAMP) * time_gap);
    speedYaw *= (1.0 - (1.0 - DAMP) * time_gap);

    return Vector {locationX,
                   locationY,
                   locationYaw,
                   speedX,
                   speedY,
                   speedYaw};
}
