//
// Created by msclab on 10/31/18.
//

#ifndef AGENTSIM_REALCARCONTROLLER_H
#define AGENTSIM_REALCARCONTROLLER_H

#include "Controller.hpp"

///
/// A RealCar controller is the controller for the real car model (RealCarModel)
class RealCarController : public Controller {
public:
    explicit RealCarController();

    Vector update(Vector input) override;
};


#endif //AGENTSIM_REALCARCONTROLLER_H
