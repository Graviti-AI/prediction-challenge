//
// Created by mscsim on 8/30/18.
//

#include "FourWheelController.hpp"
#include <stdexcept>
#include <iostream>

/// Constructor.
/// Dimension of input vector is 3 (gas pedal, brake pedal, steering angle)
/// Dimension of intermediate vector is 3.
FourWheelController::FourWheelController() : Controller(3, 3) {

}

/// Proportional controller.
/// \param input input vector (gas pedal, brake pedal, steering angle)
/// \return intermediate vector.
Vector FourWheelController::update(Vector input) {

    if (input.size() != this->dimInput) {
        throw std::runtime_error("bad dimension");
    }

    input[0] *= 1.0;
    input[1] *= 1.0;
    input[2] *= 3.0;

    // std::cout << "---------------" << std::endl;
    // std::cout << input[0] << "  " << input[1] << "  " << input[2] << std::endl;

    return input;
}
