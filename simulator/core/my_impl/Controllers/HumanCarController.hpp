//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_HUMANCARCONTROLLER_HPP
#define AGENTSIM_HUMANCARCONTROLLER_HPP


#include "Controller.hpp"

///
/// A human car controller is the controller for the kinematic car model (HumanCarModel)
class HumanCarController : public Controller {
public:
    explicit HumanCarController();

    Vector update(Vector input) override;
};


#endif //AGENTSIM_HUMANCARCONTROLLER_HPP
