//
// Created by mscsim on 7/18/19.
//

#ifndef AGENTSIM_PEDESTRIANAGENT_HPP
#define AGENTSIM_PEDESTRIANAGENT_HPP

#include <vector>
#include "../threadPool/Task.hpp"
#include "../Planners/Planner.hpp"
#include "../Controllers/Controller.hpp"
#include "../Models/Model.hpp"
//#include "../Simulator/Simulator.hpp"
typedef std::vector<double> Vector;
class Simulator;

///
/// An pedestrian agent has a state vector and an id.
/// You can get the current state, or set the next state followed by applying the next state.
class PedestrianAgent : public Task{

    friend bool operator<(PedestrianAgent  &la, PedestrianAgent &ra)
    {
        return la.priority_ < ra.priority_;
    }

public:
	PedestrianAgent(int id, Vector initialState, Vector destination);
    explicit PedestrianAgent(int id, Vector initialState, Vector destination, Planner *planner, Controller *controller, Model *model);

    int getId() const;
	bool getArrivalFlag() const;

    Vector getState() const;
    void setNextState(Vector state);
    void applyNextState();
    void setPlanner(Planner *p);
    void setController(Controller *c);
    void setModel(Model *m);
    void Run();

    int num = 0;
protected:
	bool arrivalFlag;
    Vector state;
	Vector destination;
    const int dimState;
    const int id;
    Vector nextState;
    Planner *planner; /*!< the agent's Planner */
    Controller *controller; /*!< the agent's Controller */
    Model *model; /*!< the agent's Model */
};


#endif //AGENTSIM_PEDESTRIANAGENT_HPP

