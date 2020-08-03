//
// Created by akio on 8/13/19.
//

#include "PedestrianModel.hpp"

#include "../others.hpp"

/// Constructor.
/// Dimension of state vector is 6 (x,y,yaw, speed x, speed y, speed yaw)
/// Dimension of intermediate vector is 6 (state)
PedestrianModel::PedestrianModel() : Model(6,6) {

}

/// Implements the simple kinematic model
/// \param Dimension of state vector is 6 (x,y,yaw, speed x, speed y, speed yaw)
/// \param Dimension of intermediate vector is 3 (not defined yet)
/// \return the state vector of this pedestrian for the next iteration
Vector PedestrianModel::update(Vector state, Vector intermediate){
    return intermediate;
}