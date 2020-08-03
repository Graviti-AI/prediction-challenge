//
// Created by Fan on 2018/9/26.
//

#ifndef AGENTSIM_PIDCONTROLLER_H
#define AGENTSIM_PIDCONTROLLER_H

#include "Controller.hpp"

class PidController;

///
/// A SimplePid controller is the controller for the Planner which may plan many step.
class SimplePidController : public Controller {
public:
    explicit SimplePidController();

    virtual ~SimplePidController();

    Vector update(Vector input) override;

protected:
    PidController *innerController;
};


#endif //AGENTSIM_PIDCONTROLLER_H
