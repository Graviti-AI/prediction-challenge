//
// Created by msclab on 10/31/18.
//

#include "RealCarController.hpp"
#include <stdexcept>
#include <iostream>

/// Constructor.
/// Dimension of input vector is 3 (gas pedal, brake pedal, steering angle)
/// Dimension of intermediate vector is 3.
RealCarController::RealCarController() : Controller(3, 3) {

}

/// Proportional controller.
/// \param input input vector (gas pedal, brake pedal, steering angle)
/// \return intermediate vector which is empty,since the real car's state is coming form the real world.
Vector RealCarController::update(Vector input) {

    if (input.size() != this->dimInput) {
        throw std::runtime_error("bad dimension");
    }

    input[0] *= 0.0;
    input[1] *= 0.0;
    input[2] *= 0.0;

    std::cout << input[0] << "  " << input[1] << "  " << input[2] << std::endl;

    return input;
}
