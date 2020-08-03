//
// Created by akio on 8/13/19.
//

#ifndef AGENTSIM_PEDESTRIANCONTROLLER_HPP
#define AGENTSIM_PEDESTRIANCONTROLLER_HPP


#include "Controller.hpp"

///
/// A pedestrian controller
class PedestrianController : public Controller {
public:
    explicit PedestrianController();

    Vector update(Vector input) override;
};

#endif //AGENTSIM_PEDESTRIANCONTROLLER_HPP
