//
// Created by akio on 8/13/19.
//

#ifndef AGENTSIM_PEDESTRIANMODEL_HPP
#define AGENTSIM_PEDESTRIANMODEL_HPP

#include "Model.hpp"

///
/// Simple kinematic model for a pedestrian
class PedestrianModel : public Model {
public:
    explicit PedestrianModel();

    Vector update(Vector state, Vector intermediate) override;

};


#endif //AGENTSIM_PEDESTRIANMODEL_HPP