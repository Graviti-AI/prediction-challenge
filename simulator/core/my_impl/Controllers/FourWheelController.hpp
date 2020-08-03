//
// Created by mscsim on 8/30/18.
//

#ifndef AGENTSIM_FOURWHEELCONTROLLER_HPP
#define AGENTSIM_FOURWHEELCONTROLLER_HPP


#include "Controller.hpp"

///
/// A four-wheel controller is the controller for the complex car model (FourWheelModel)
class FourWheelController : public Controller {
public:
    explicit FourWheelController();

    Vector update(Vector input) override;
};


#endif //AGENTSIM_FOURWHEELCONTROLLER_HPP
