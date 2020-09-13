#include <iostream>
#include <assert.h>
#include "my_simulator_impl.hpp"

#include "Simulator/Simulator.hpp"
#include "threadPool/MyThreadPool.hpp"
#include "Controllers/PedestrianController.hpp"
#include "Models/PedestrianModel.hpp"
#include "Planners/PedestrianPlanner.hpp"
#include "Planners/HumanPedestrianPlanner.hpp"


using namespace core;



MySimulatorImpl::MySimulatorImpl(int rviz_port):simulator(rviz_port)
{
    
}

MySimulatorImpl::~MySimulatorImpl()
{

}

void MySimulatorImpl::start(const std::string& config_file)
{
    std::cout << "Simulator Runs with configuration file " << config_file<<std::endl;

    //If config_file == "", then each car will be randomly generated
        simulator.InitSimulation(config_file);
    
    //If you don't want to add other cars, please omit this line. 
        simulator.run();
}

bool MySimulatorImpl::onUserState(std::vector<Trajectory> pred_trajs, std::vector<double> probability)
{
    assert(pred_trajs.size() == probability.size());
    uint64_t car_id = pred_trajs[0][0]->track_id;

    // check trajs
    for (auto traj : pred_trajs){
        assert(traj.size() == 30);
        for (auto s : traj)
            assert(s->track_id == car_id);
    }

    // check prob
    double sum_prob = 0.0;
    for (auto p : probability)
        sum_prob += p;
    
    assert(abs(sum_prob - 1.0) < 1e-2);

    printf("\n### Receive Traj from Client, number = %d, Car ID = %d\n", (int)pred_trajs.size(), (int)car_id);

    for (int i = 0; i < pred_trajs.size(); i ++){
        auto s = pred_trajs[i][29];
        printf("# Traj: %d; Prob: %.3lf; frame_id: %d; x: %.3lf; y: %.3lf\n", i, probability[i], (int)s->frame_id, s->x, s->y);
    }

    simulator.upload_traj(car_id, pred_trajs, probability);
    return true;
}

core::SimulationEnv MySimulatorImpl::fetchEnv()
{
    core::SimulationEnv env = simulator.fetch_history();

    assert(env.my_traj.size() == 10);
    for (auto t : env.other_trajs)
        assert(t.size() == 10);

    return env;
}

void MySimulatorImpl::shutdown()
{

}
