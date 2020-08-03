

#include "NewVirtualCar.hpp"
#include "../Simulator/Simulator.hpp"


using namespace std;
/// Constructor.
/// \param id id of the new virtual car.
/// \param initialState initial state of the virtual car.
NewVirtualCar::NewVirtualCar(int id, Vector initialState, FSM* b, Planner *p, Controller *c, Model *m)
        : Agent(id, initialState) {
        fsm = b;
        // planner = p;
        controller = c;
        model = m;
}

NewVirtualCar::NewVirtualCar(int id, Vector initialState)
        : Agent(id, initialState) {
        first_run = true;
}

NewVirtualCar::~NewVirtualCar()
{
}

Vector NewVirtualCar::Cruise(const Vector& currentState, const Vector &humanInput, std::vector<Agent*> agents){
    Vector targetState(7, 0.0);
    // double acceleration;
    
    // std::clock_t current_time = std::clock();
    // double time_gap = (current_time - last_time) / (double) CLOCKS_PER_SEC;
    
    // last_time = current_time;
    // BasicPoint2d pBefore(currentState[0], currentState[1]);
    // time_gap = 0.02; // TODO, use the real time
    // double v_before = currentState[3];
    // acceleration = params.maxAcc * (1 - pow((v_before/8.0),4));
    // double v = v_before + acceleration * time_gap;
    // v = fmax(v, 0.0); // In case the velocity is negative
    // // globleS = geometry::toArcCoordinates(purePursuitPath, pBefore).length + v * time_gap;

    // s_ += v * time_gap;
   
    // // if (v < 0.5) pBefore = geometry::interpolatedPointAtDistance(currentLanelet_.centerline2d(), s_ - 1.0);
    // BasicPoint2d pAfter = geometry::interpolatedPointAtDistance(currentLanelet_.centerline2d(), s_);
    // // cout<<pAfter - pBefore<<endl;
    // BasicPoint2d pDirection = pAfter - pBefore;
    // targetState[0] = pAfter.x();
    // targetState[1] = pAfter.y();
    // targetState[2] = -std::atan2(pDirection.x(), -pDirection.y());
    // targetState[3] = v;
    // targetState[6] = acceleration;

    return targetState;
}

Vector NewVirtualCar::Intersection_pass(Vector& currentState, Vector &humanInput, std::vector<Agent*> agents){
    Vector nextState(7, 0.0);

    std::chrono::time_point<std::chrono::system_clock> init_time = std::chrono::system_clock::now();
    // vector<Agent* > OtherAgent = fsm->getTargetAgents();
    BasicPoint2d currPos(currentState[0], currentState[1]);
    this->s_ = geometry::toArcCoordinates(fsm->wholePath, currPos).length;
    cout<<"car's"<<this->s_<<endl;
    auto tmpPlannerResult = planner->update(currentState, fsm->spl_x,fsm->spl_y, humanInput, agents, this->s_);
    Vector plannerResult(7, 0.0);
    
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();        
    double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
    // int frame_id = 3;
    int frame_id = spinnedTime/10+1;
    // cout<<"frame_id"<<frame_id<<endl;
    plannerResult[0] = tmpPlannerResult[1 + 5*frame_id];
    plannerResult[1] = tmpPlannerResult[2 + 5*frame_id];
    plannerResult[3] = tmpPlannerResult[3 + 5*frame_id];
    plannerResult[2] = tmpPlannerResult[4 + 5*frame_id];
    plannerResult[0] *= 100.0;
    plannerResult[1] *= -100.0;

    if (plannerResult[3] > 10) {
        std::cout << "Before Astar velocity : " << currentState[3] << std::endl;
        std::cout << "Astar velocity : " << plannerResult[3] << std::endl;
    }

    
    nextState = plannerResult;

    return nextState;
}


void NewVirtualCar::Run() {


    std::chrono::time_point<std::chrono::system_clock> i_time = std::chrono::system_clock::now();
    if (isRunning) {
        return;
    } else {
        isRunning = true;
    }
    std::vector<Agent *> agents =  Simulator::agentsForThread;
    Vector humanInput;
    Vector curState = this->getState();

    if(curState.empty()) {
        this->hasReachedDestinaiton = true;
        isRunning = false;
        return;
    }
    curState[0] /= 100.0;
    curState[1] /= -100.0;

    double dest_x = fsm->wholePath.back().x();
    double dest_y = fsm->wholePath.back().y();
    double dis = sqrt(pow(dest_x-curState[0], 2.0) + pow(dest_y-curState[1], 2.0));
    if(dis < 15.0) {
        this->hasReachedDestinaiton = true;
        isRunning = false;
        return;
    }

    Vector nextState(7, 0.0);
    int currMode = fsm->getMode();
    

    local_s = local_s + s_ - last_s_;
    ConstLanelet currentLanelet = fsm->getCurrentLanelet();
    ConstLanelet nextLanelet = fsm->getNextLanelet();
    if (local_s > geometry::length2d(currentLanelet)) {
        fsm->moveToLanelet(nextLanelet); // Reaches the end
        local_s=local_s-geometry::length2d(currentLanelet);
    }
    last_s_=s_;


    cout<<"-------------lane id--------  "<<currentLanelet.id()<<" ------- local_S"<<local_s<<" ------- current mode"<<currMode<<endl;
    // if (true) {
    //     std::cout << "Velocity : " << curState[3] << std::endl;
    //     std::cout << "FSMMode : " << currMode << std::endl;
    //     //std::cout << "hasReachedDestinaiton : " << hasReachedDestinaiton << std::endl;
    // }
    // cout<<"agent size is "<<agents.size()<<endl;
    // std::chrono::time_point<std::chrono::system_clock> i_time = std::chrono::system_clock::now();

    // Vector FSMResultState = fsm->Update(curState, humanInput, agents);
    
    // if (FSMResultState.empty()) {
    //     hasReachedDestinaiton = true; // Reached end
    //     cout<<"out"<<endl;
    //     state = FSMResultState;
    //     isRunning = false;
    //     return;

    // } else if (currMode == FSMMode::merge|| currMode == FSMMode::intersection_pass  ) {
    if (true) {
    // } else if (currMode == FSMMode::merge) {
        std::chrono::time_point<std::chrono::system_clock> init_time = std::chrono::system_clock::now();
        vector<Agent* > OtherAgent = fsm->getTargetAgents();


        nextState=Intersection_pass(curState,humanInput,OtherAgent);


        // BasicPoint2d currPos(curState[0], curState[1]);
        // this->s_ = geometry::toArcCoordinates(fsm->wholePath, currPos).length;
        // cout<<"car's"<<this->s_<<endl;
        // auto tmpPlannerResult = planner->update(curState, fsm->spl_x,fsm->spl_y, humanInput, OtherAgent, this->s_);
        // Vector plannerResult(7, 0.0);
        
        // std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();        
        // double spinnedTime =  std::chrono::duration_cast<std::chrono::milliseconds>(current_time - init_time).count();
        // int frame_id = 3;
        // // int frame_id = spinnedTime/10+1;
        // // cout<<"frame_id"<<frame_id<<endl;
        // plannerResult[0] = tmpPlannerResult[1 + 5*frame_id];
        // plannerResult[1] = tmpPlannerResult[2 + 5*frame_id];
        // plannerResult[3] = tmpPlannerResult[3 + 5*frame_id];
        // plannerResult[2] = tmpPlannerResult[4 + 5*frame_id];
        // plannerResult[0] *= 100.0;
        // plannerResult[1] *= -100.0;

        // if (plannerResult[3] > 10) {
        //     std::cout << "Before Astar velocity : " << curState[3] << std::endl;
        //     std::cout << "Astar velocity : " << plannerResult[3] << std::endl;
        // }

        
        // nextState = plannerResult;
    } 
    // else {
    //     BasicPoint2d currPos(curState[0], curState[1]);
    //     this->s_ = geometry::toArcCoordinates(fsm->wholePath, currPos).length;
    //     FSMResultState[2] = std::atan2(-FSMResultState[1] + curState[1], FSMResultState[0] - curState[0]);
    //     if (fsm->getCurrentLanelet().id() == 30400 || fsm->getCurrentLanelet().id() == 30402) {
    //         FSMResultState[2] = curState[2];
    //     }

    //     // std::chrono::time_point<std::chrono::system_clock> c_time = std::chrono::system_clock::now();
    //     // double delay=  std::chrono::duration_cast<std::chrono::milliseconds>(c_time - i_time).count();
    //     // int frame_id2=delay/10 + 1;
    //     // cout<<"the frame in fsm update: "<<frame_id2<<endl;

    //     // fsm->Add_s( FSMResultState[3]*0.01*frame_id2 );


    //     FSMResultState[0] *= 100.0;
    //     FSMResultState[1] *= -100.0;
    //     nextState = FSMResultState;
    // }
   
    if (state[0] == 5000.0) {
        state = nextState;
        isRunning = false;
        return;
    }

    state = nextState;
        // std::cout << "Simulator Run" << std::endl;
// tmp
    if (state[3] < 0.1) {
        state[2] = curState[2];
    }
    
//     Vector nextState = model->update(this->getState(), intermediate);
    // this->setNextState(nextState); // Set the next state to apply, but not apply right now.
//     this->setPreState(this->getState());
    // this->applyNextState();
    isRunning = false;


}

/// Type getter (overridden)
/// \return type (virtual car)
AgentType NewVirtualCar::getType() const {
    return AgentType::NewVirtualCar;
}

FSM* NewVirtualCar::getFSM()  {
    return this->fsm;
}
