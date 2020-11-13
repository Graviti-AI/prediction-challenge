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
///
/// class of Astar method
class SpeedProfAstarSolver
{

public:
	/// @param[in] s0 initial position
	/// @param[in] v0 initial velocity
	/// @param[in] horizon look ahead horizon
	/// @param[in] dt environment information
	SpeedProfAstarSolver(const double s0, const double v0, const int horizon, const float dt, vector<int> actions, vector<double> PATH1);

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
		Vehicle(const double s_f, const double s_b, const double v, const double t_i,const double t_o) :
			s_f_(s_f),
			s_b_(s_b),
			v0_(v),
			t_i_(t_i),
			t_o_(t_o),
			//run_over_(false),
            follow_(false)
		{}
        double s_f_;
        double s_b_;
        double v0_;
        double t_i_;
        double t_o_;
        //double s_;
        //double v_;
        //bool run_over_;
        bool follow_;
        /*inline void updateS(const double a, const double dt)
		{
			this->s_ = this->s_ + this->v_*dt + 1 / 2 * a*dt*dt;
		}
		inline void updateV(const double a, const double dt)
		{
			this->v_ = this->v_ + a*dt;
		}*/
	};
	/// The cross bound
	class Cross
	{
	public:
		Cross(const double s, const double t, const CrossType type) :
			s_(s),
			t_(t),
			type_(type)
		{}
        double s_;
        double t_;
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

	/// Sampling time
    double dt_;

	/// List of explored states that are not closed yet.
	MinHeap frontier_;

	/// Tags
	boost::multi_array<Tag, 3> t_;

	/// Initial position
    double s0_;

	/// Initial velocity
    double v0_;

	/// Start node
	Node start_node_;

	/// Preview horizon
	int horizon_;

	/// The accelerations
	vector<int> actions_;

	vector<double> PATH1_;


    /// weight of cost
    double w_a_;
    /// weight of cost
    double w_an_;
    /// weight of cost
    double w_v_;
    /// weight of cost
    double desiredV_;
    /// curvature data
    //double curvature_[12500];
    double * curvature_;
    double w_change_;
    double w_runover_;





	/// get actions number
	inline int actionNum()
	{
		return sizeof(this->actions_);
	}

	/// Compute traverse cost from a node to its successor
	/// @param[in] node current state
	/// @param[in] succ its successor
	/// @return traverse cost, currently just euclidean distance
    double getCost( const Node& node, double a, const double path1,vector<int> &passornot, double *COSTT);
    
    //vector<int > getPassornot(const Node& node, double a, const double path1);

	/// Explore a leaf node.
	/// Operations include 1) find successors, 2) assign itself as parent of successors, 3) compute their attributes: path cost, heuristic cost and total cost
	/// @param[in] node query node
	/// @param[in] path_cost path cost so far
	/// @return vector of expanded successors
	std::vector<Node> expandLeafNode(const Node& node, vector<int> actions, const double path1);

	/// chenck goal state
	bool checkGoalState(const Node& node, int goalType);
	/// get the path
	int* getPath(const Node& node);

	//double getCOSTT(const Node& node, double a, const double path1);
};
#endif
