//
// Created by LCR on 7/18/20.
//


#include "BehaveCar.hpp"
#include "../Simulator/Simulator.hpp"

/// Constructor.
/// \param id id of the new human car.
/// \param initialState initial state of the human car.
BehaveCar::BehaveCar(int id, Vector initialState, Planner *planner, Controller *controller, Model *model)
        : Agent(id, initialState) {

}

BehaveCar::BehaveCar(int id, Vector initialState)
        : Agent(id, initialState) {

}
void BehaveCar::Run() {
    // Get the input vector from the planner, by giving human inputs and agent state.
    if (isRunning||hasReachedDestinaiton) {
        return;
    } else {
        isRunning = true;
    }
    vector<Agent *> agents =  Simulator::agentsForThread;
    // Behaviour
    //std::cout<<"behave begin!"<<endl;

    //std::cout << "## DEBUG | Waiting for Behaviour, ID: " << getId() << endl;

    std::chrono::time_point<std::chrono::system_clock> inrun_time = std::chrono::system_clock::now();
    Vector Behavestate = behaviour->update(this->getState(), Simulator::humanInputsForThread[this], agents);
    std::chrono::time_point<std::chrono::system_clock> behave_time = std::chrono::system_clock::now();

    //std::cout<<"## DEBUG | Behaviour updated! current mode: "<<behaviour->getMode() << " ID: " << getId() <<endl;

    Vector nextState = this->getState();
    //cout<<"\n#### state: ";
    for (int i = 0;i<6;i++) {
        nextState[i] = Behavestate[i];
        //cout<<nextState[i]<<" ";
    }
    //cout<<endl;
    
    if (behaviour->getMode()==Mode::following && !IDM_){
        std::chrono::time_point<std::chrono::system_clock> init_time = std::chrono::system_clock::now();
        std::vector<double> tmpPlannerResult = fplanner->update(this->getState(), Simulator::humanInputsForThread[this], agents, behaviour->obstacles_info_);
        std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();        
        double planningtime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
        double wholetime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - inrun_time).count();
        double behavetime =  std::chrono::duration_cast<std::chrono::milliseconds>(behave_time - inrun_time).count();
        cout<<"Whole time: "<<wholetime<<" ms "<<"behave time: "<<behavetime<<" ms "<<"planning time: "<<planningtime<<" ms "<<endl;
        int frame_id = 1; //wholetime/10+1;
        nextState[0] = tmpPlannerResult[1 + 5*frame_id];
        nextState[1] = tmpPlannerResult[2 + 5*frame_id];
        nextState[3] = tmpPlannerResult[3 + 5*frame_id];
        nextState[2] = tmpPlannerResult[4 + 5*frame_id];       
        //cout<<"x: "<<nextState[0]<<" y: "<<nextState[1]<<" v: "<<nextState[3]<<" theta: "<< nextState[2]<<endl;
    }

    mapinfo->update(nextState);
    //std::cout<<"mapinfo update"<<endl;
    //cout<<"current lanelet id: "<< mapinfo->getCurrentLaneletId()<<" current s: "<<mapinfo->getS()<<endl;
    //cout<<"next lanelet id: "<< mapinfo->getNextLanelet()<<endl;
    ConstLanelet next_lanelet = mapinfo->getNextLanelet();
    //cout<<"next next lanelet id: "<< mapinfo->findNextLanelet(next_lanelet)<<endl;
    //cout<<"current RoutingLineChange? "<<mapinfo->RoutingLineChange_<<endl;
    //cout<<"current hasReachedDestinaiton? "<<mapinfo->HasArrivedDestination_<<endl;

    if (mapinfo->HasArrivedDestination_) {
        hasReachedDestinaiton = true;
        this->setNextState(nextState); // Set the next state to apply, but not apply right now.
        this->setPreState(this->getState());
        this->applyNextState();
        PredictTra_ = predictor->update(nextState, agents);

        printf("Behavior Car (%d) has arrived destination!\n", getId());
        isRunning = false;
        return;
    }
    
    this->setNextState(nextState); // Set the next state to apply, but not apply right now.
    this->setPreState(this->getState());
    this->applyNextState();
    //std::chrono::time_point<std::chrono::system_clock> end_loop_time = std::chrono::system_clock::now();
    //double prediction_time =  std::chrono::duration_cast<std::chrono::milliseconds>(end_loop_time - prediction_begin_time).count(); 
    //double looptime =  std::chrono::duration_cast<std::chrono::milliseconds>(end_loop_time - inrun_time).count();
    //cout<<"predition time: "<<prediction_time<<endl;
    //cout<<"loop time: "<<looptime<<endl;
    //cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;

    //std::chrono::time_point<std::chrono::system_clock> prediction_begin_time = std::chrono::system_clock::now(); 
    PredictTra_ = predictor->update(nextState, agents);
    /*  
    cout<<"PredictTra_: "<<endl;
    for(auto &one: PredictTra_.Trajs) {
        cout<<"***************"<<endl;
        cout<<"Probability: "<<one.Probability<<endl;
        cout<<"confilictlanes: "<<endl;
        for(auto &ll: one.confilictlanes) cout<<ll<<endl;
        cout<<"trajectory: "<<endl;
        for(auto &point: one.Traj){
            cout<<"-------"<<endl;
            cout<<"t: "<<point.t<<" "<<endl;
            cout<<"x: "<<point.x<<" "<<endl;
            cout<<"y: "<<point.y<<" "<<endl;
            cout<<"theta: "<<point.theta<<" "<<endl;
            cout<<"v: "<<point.v<<" "<<endl;
            cout<<"current lanelet: "<<point.current_lanelet<<" "<<endl;
            cout<<"s of current lanelet: "<<point.s_of_current_lanelet<<" "<<endl;
            cout<<"d of current lanelet: "<<point.d_of_current_lanelet<<" "<<endl;
        }
    }
    */

    isRunning = false;
}
/// Type getter (overridden)
/// \return type (human car)
AgentType BehaveCar::getType() const {
    return AgentType::BehaveCar;
}