//
// Created by mscsim on 8/13/19.
//

#include "PedestrianController.hpp"
#include <stdexcept>
#include <iostream>

/// Constructor.
/// Dimension of input vector is 6 (state)
/// Dimension of intermediate vector is 6.
PedestrianController::PedestrianController() : Controller(6, 6) {

}

/// Proportional controller.
/// \param input input vector (not defined yet)
/// \return intermediate vector.
Vector PedestrianController::update(Vector input) {

    if (input.size() != this->dimInput) {
        throw std::runtime_error("bad dimension");
    }

    //std::cout << "---------------" << std::endl;
    //std::cout << input[0] << "  " << input[1] << "  " << input[2] << std::endl;//

    return input;
}