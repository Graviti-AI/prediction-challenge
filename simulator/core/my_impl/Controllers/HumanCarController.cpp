//
// Created by mscsim on 8/23/18.
//

#include "HumanCarController.hpp"
#include <stdexcept>

/// Constructor.
/// Dimension of input vector is 3 (gas pedal, brake pedal, steering angle)
/// Dimension of intermediate vector is 3.
HumanCarController::HumanCarController() : Controller(3, 3) {

}

/// Proportional controller.
/// \param input input vector (gas pedal, brake pedal, steering angle)
/// \return intermediate vector.
Vector HumanCarController::update(Vector input) {

    if (input.size() != this->dimInput) {
        throw std::runtime_error("bad dimension");
    }

    input[0] *= 160;
    input[1] *= 640;
    input[2] *= 3.0;

    return input;
}
