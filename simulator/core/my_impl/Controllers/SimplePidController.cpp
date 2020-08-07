//
// Created by Fan on 2018/9/26.
//

#include "SimplePidController.hpp"
#include "PidControllerLib/PidController.h"
#include "../simulator_state.hpp"

#define PLAN_STEP (10)

/// Constructor.
/// Dimension of input vector is 6 + 4 * PLAN_STEP (current state and PLAN_STEP state(t, x , y, v) that want to follow)
/// Dimension of intermediate vector is 3.
SimplePidController::SimplePidController() : Controller(6 + 4 * PLAN_STEP, 3) {
    this->innerController = new PidController(SIM_TICK);
}

/// Proportional controller.
/// \param input input vector (current state and PLAN_STEP state(t, x , y, v) that want to follow)
/// \return intermediate vector.
Vector SimplePidController::update(Vector input) {

    Vector state = Vector(6);
    Vector planningResult = Vector(4 * PLAN_STEP);

    for (int i = 0; i < 6; i++) {
        state[i] = input[i];
    }

    for (int i = 6; i < 6 + 4 * PLAN_STEP; i++) {
        planningResult[i - 6] = input[i];
    }

    return this->innerController->internalUpdate(state, planningResult);
}

SimplePidController::~SimplePidController() {
    delete this->innerController;
}
