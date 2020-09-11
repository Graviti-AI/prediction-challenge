//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_AGENT_HPP
#define AGENTSIM_AGENT_HPP

#include <vector>
#include "../threadPool/Task.hpp"
#include "../Planners/Planner.hpp"
#include "../Controllers/Controller.hpp"
#include "../Models/Model.hpp"
#include "../Behaviours/Behaviour.hpp"
#include "../Predictors/Predictor.hpp"

//#include "../Simulator/Simulator.hpp"
typedef std::vector<double> Vector;
class Simulator;

///
/// This enumerator stands for the type of an agent.
enum AgentType {
    HumanCar = 0,
    VirtualCar = 1,
    RealCar = 2,
    ReplayCar,
    NewVirtualCar,
    BehaveCar
};


/// The parent class of all agents. An agent has a state vector and an id.
/** You can get the current state, or set the next state followed by applying the next state.*/
class Agent : public Task{

    friend bool operator<(Agent  &la,Agent &ra)
    {
        return la.priority_ < ra.priority_;
    }

public:
    Agent(int id, Vector initialState);
    explicit Agent(int id, Vector initialState, Planner *planner, Controller *controller, Model *model);

    int getId() const;
    double s_;
    Vector getState() const;
    std::vector<Vector>  get_preState() const;

    void setState(Vector st);
    void setNextState(Vector state);
    void setPreState(Vector state);
    void applyNextState();
    void setPlanner(Planner *p);
    void setfollowingPlanner(Planner *p);
    void setlinechangePlanner(Planner *p);
    void setController(Controller *c);
    void setModel(Model *m);
    void setBehaviour(Behaviour *b);
    void setMapinfo(MapInfo *m);
    void setPredictor(Predictor *p);
    Predictor* getPredictor();
    virtual AgentType getType() const = 0;
    //void Run(Planner* planner, Controller* controller, Model* model, std::vector<Agent*> agents, Vector &humanInputs);
    void Run();
    double length_ = 4;
    double width_ = 2;
    int num = 0;
    bool hasReachedDestinaiton = false;
    bool isRunning = false;
    PredictTra PredictTra_;
    MapInfo* mapinfo;
    
protected:
    Vector state; /*!< the current state of the agent */
    std::vector<Vector>  preState; /*!< pre state to get a*/
    const int dimState; /*!< the dimension of the agent state */
    const int id; /*!< the agent's ID */
    Vector nextState; /*!< next state which is calculated from model and waiting for apply */
    Planner *planner;
    Planner *lplanner; /*!< the agent's Planner */
    Planner *fplanner; /*!< the agent's Planner */
    Controller *controller; /*!< the agent's Controller */
    Model *model; /*!< the agent's Model */
    Behaviour* behaviour;
    Predictor* predictor;
};


#endif //AGENTSIM_AGENT_HPP
