//
// Created by mscsim on 12/13/18.
//

#ifndef AGENTSIM_VIRTUALCARMODEL_H
#define AGENTSIM_VIRTUALCARMODEL_H

#include "Model.hpp"

///
/// model for the virtual car
class VirtualCarModel : public Model{
public:
    explicit VirtualCarModel();

    Vector update(Vector state, Vector intermediate) override;

};


#endif //AGENTSIM_VIRTUALCARMODEL_H
