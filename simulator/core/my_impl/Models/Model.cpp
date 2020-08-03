//
// Created by mscsim on 8/23/18.
//

#include "Model.hpp"

/// Constructor.
/// \param dimState dimension of the state vector.
/// \param dimIntermediate dimension of the intermediate vector.
Model::Model(int dimState, int dimIntermediate)
: dimState(dimState), dimIntermediate(dimIntermediate) {
    last_time = std::clock();
}
