//
// Created by mscsim on 8/23/18.
//

#include "Controller.hpp"

/// Constructor.
/// \param dimInput dimension of the input vector, which is constant for a certain type of controllers.
/// \param dimIntermediate dimension of the intermediate vector, which is constant for a certain type of controllers.
Controller::Controller(int dimInput, int dimIntermediate)
: dimInput(dimInput), dimIntermediate(dimIntermediate) {

}
