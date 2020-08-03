//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_HUMANCARMODEL_HPP
#define AGENTSIM_HUMANCARMODEL_HPP

#include "Model.hpp"

///
/// Simple kinematic model for a human car
class HumanCarModel : public Model {
public:
    explicit HumanCarModel();

    Vector update(Vector state, Vector intermediate) override;

};


#endif //AGENTSIM_HUMANCARMODEL_HPP
