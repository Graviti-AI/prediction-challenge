#ifndef _ASTAR_SOLVER_H
#define _ASTAR_SOLVER_H


#include <string>
#include <queue>
#include <vector>
#include "boost/multi_array.hpp"
#include "MinHeap.h"
#include <time.h>
#include <array>
using namespace std;

class SpeedProfAstarSolver
{

public:
    /// @param[in] s0 initial position
    /// @param[in] v0 initial velocity
    /// @param[in] horizon look ahead horizon
    /// @param[in] dt environment information
    ///SpeedProfAstarSolver(const vector<double> curvature,const float s0, const float v0, const int horizon, const float dt, vector<int> actions);

    SpeedProfAstarSolver(const double* curvature,const float s0, const float v0, const int horizon, const float dt, vector<int> actions);
    /// Plan the best path connecting start pose -> goal pose
    /// return 0 if success, 1 if failure
    Node plan();


    enum Tag
    {
        /// Never visited
                New = 0,
        /// In openlist
                Open = 1,
        /// Explored
                Explored = 2
    };
    enum CrossType
    {
        Stop = 0,
        Pass = 1
    };
    /*enum VehicleLane
    {
        /// In same lane
        Same = 0,
        /// In left lane
        Left = 1,
        /// In right lane
        Right = 2
    };*/

    /// The surrouding vehicle
    class Vehicle
    {
    public:
        Vehicle(const double s, const double v, const double t) :
                s0_(s),
                v0_(v),
                t0_(t),
                run_over_(false),
                follow_(false)
        {}
        double s0_;
        double v0_;
        double t0_;
        double s_;
        double v_;
        bool run_over_;
        bool follow_;
        inline void updateS(const double a, const double dt)
        {
            this->s_ = this->s_ + this->v_*dt + 1 / 2 * a*dt*dt;
        }
        inline void updateV(const double a, const double dt)
        {
            this->v_ = this->v_ + a*dt;
        }
    };
    /// The cross bound
    class Cross
    {
    public:
        Cross(const float s, const float t, const CrossType type) :
                s_(s),
                t_(t),
                type_(type)
        {}
        float s_;
        float t_;
        CrossType type_;
        inline void setType(const CrossType type)
        {
            this->type_ = type;
        }
    };
    /// The environment
    class Environment
    {
    public:
        Environment(){};
        Environment(const std::vector<Vehicle> Vehicles, const std::vector<Cross> Crosses) :
                Vehicles_(Vehicles),
                Crosses_(Crosses)
        {}
        std::vector<Vehicle> Vehicles_;
        std::vector<Cross> Crosses_;
        void AddVehicle(const Vehicle vehicle)
        {
            Vehicles_.push_back(vehicle);
        }
        void AddCross(const Cross cross)
        {
            Crosses_.push_back(cross);
        }
    } env_;
    //the curvature in the environment
    const double *curvature;

    /// Sampling time
    float dt_;

    /// List of explored states that are not closed yet.
    MinHeap frontier_;

    /// Tags
    boost::multi_array<Tag, 3> t_;

    /// Initial position
    float s0_;

    /// Initial velocity
    float v0_;

    /// Start node
    Node start_node_;

    /// Preview horizon
    int horizon_;

    /// The accelerations
    vector<int> actions_;
    /// Desired value
    float desiredV=8;
    float w_a=0;
    float w_an=0;
    float w_v=1;
    /// get actions number
    inline int actionNum()
    {
        return sizeof(this->actions_);
    }

    /// Compute traverse cost from a node to its successor
    /// @param[in] node current state
    /// @param[in] succ its successor
    /// @return traverse cost, currently just euclidean distance
    float getCost(const Node& node, double action);

    /// Explore a leaf node.
    /// Operations include 1) find successors, 2) assign itself as parent of successors, 3) compute their attributes: path cost, heuristic cost and total cost
    /// @param[in] node query node
    /// @param[in] path_cost path cost so far
    /// @return vector of expanded successors
    std::vector<Node> expandLeafNode(const Node& node, vector<int> actions);

    /// chenck goal state
    bool checkGoalState(const Node& node, int goalType);
    /// get the path
    int* getPath(const Node& node);
};
#endif
