//
// Created by mscsim on 12/13/18.
//

#ifndef AGENTSIM_VIRTUALCARCONTROLLER_H
#define AGENTSIM_VIRTUALCARCONTROLLER_H

#include "Controller.hpp"

///
/// A VirtualCar controller is the controller for the virtual car model (VirtualCarModel)
class VirtualCarController : public Controller{
public:
    explicit VirtualCarController();

    Vector update(Vector input) override;
};


#endif //AGENTSIM_VIRTUALCARCONTROLLER_H
