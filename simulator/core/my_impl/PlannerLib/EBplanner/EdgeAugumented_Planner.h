#pragma once
#include <vector>
#include "ElasticBand.h"
#include "Global_parameter.h"
using namespace std;

typedef pair<int, int> pair_i;
typedef pair<double, double> pair_d;

struct _Reference{
	double _x;
	double _y;
	double _dx;
	double _dy;
	double _s;
	_Reference(void){};
};


struct pathPoint {
	double _x;
	double _y;
	double _theta;
	double _v = MAX_SPEED;
	double _s;   // arc length
	double _curvature;
	double _dmin; // min distance to obstacles
	pathPoint(void) {};
	pathPoint(double x, double y, double theta) {
		_x = x;
		_y = y;
		_theta = theta;
	}
};

struct obstacle_ {
	double _x;
	double _y;
	double _r;
	obstacle_ (void) {}
	obstacle_(pair_d p, double r) {
		_x = p.first;
		_y = p.second;
		_r = r;
	}
};

struct car_obstacle_ {
	double _x0;
	double _y0;
	double _x1;
	double _y1;
	double _x2;
	double _y2;
	double _theta;
	double _length;
	double _width;
	double _radius;
    double _vx;
    double _vy;
	double _possibility;
	bool _followcar;
    pair_d _intersection_point;
	double curvature;
	car_obstacle_(void) {};
};

// one spatial node have IN*OUT augumented nodes, these nodes have the same repulsion and attraction force
struct SpatialNode {
	double IN;
	double OUT;
	double _x;
	double _y;
	double _r;
	double _layer;		// the node is in the _layer th layer
	double _layer_num;  // the _layer_num th row of this layer
	force_ _repulsion;   
	force_ _attraction;
	bool  _feasibility;
	//vector<AugumentedNode> _augumentedNode;
	SpatialNode(void) {};
	SpatialNode(double in,double out,double x,double y,double layer,double layer_num) {
		IN = in;
		OUT = out;
		_x = x;
		_y = y;
		_layer = layer;
		_layer_num = layer_num;
		_repulsion = force_();
		_attraction = force_();
	}
};

struct AugumentedNode {
	double _x;
	double _y;
	int InID;
	int OutID;
	double _spatialNodeID;
	double Force2;
	double minCostToEnd;
	Force _force;
	bool  feasibility;
	AugumentedNode(void) {};
};

class EdgeAugumented_Planner {
public:
	int	N_column = N_COLUMN;  // number of column of spatial graph
	int	N_row = N_ROW;		// number of row of spatial graph
    double delta_row;
	int	edge_number = EDGE_NUMBER;
    double	road_width = ROAD_WIDTH ;
	double max_radius_ = MAX_RADIUS_;  // max radius of each spatial node
	double min_distance_to_obstacles = MIN_DISTANCE_TO_OBSTACLES;  // the min distance to obstacels we permit,
											//*nodes whose distance is smaller than this value should be deleted during update
	double attract_force_gain_ = ATTRACT_FORCE_GAIN_;
	double repulisive_force_gain_ = REPULISIVE_FORCE_GAIN_;
    double repulisive_force_speed_gain = REPULISIVE_FORCE_SPEED_GAIN;
	double contraction_force_gain_ = CONTRACTION_FORCE_GAIN_;
	double Step_spatialnode = STEP_SPATIALNODE;
	int best_path[20];    // <---------Need to be same as N_column

	int start_point_layernum;	// update after a circle
	pair_d start_point_real;	// update after a circle
    pair_d end_point_real;
	vector<SpatialNode> spatialLayer;
	vector<vector<SpatialNode>> spatialGraph;

	vector<AugumentedNode> AugumentedNodes;		// save augumented nodes whose middle spatial node are same
	vector<vector<AugumentedNode>> AugumentedLayer;
	vector<vector<vector<AugumentedNode>>> AugumentedGraph;

	vector<obstacle_> obstacleSet;
    vector<vector<car_obstacle_>> CarObstacleWithinSafeDistance;
	vector<pathPoint> path;
	vector<vector<car_obstacle_>>  CarObstacleSet;

	vector<_Reference>    PathReference;

	void SimplePathPlanner();
	void initialization_SpatialGraph();
	void initialize_EdgeAugumented();
	bool find_min_cost();
	bool find_best_path();
	bool update_SpatialGraph();
	void calcPathTheta();
	bool calcMinDistanceBetweenObstacles(int layer, int layer_num);
	bool calcObstacleKinematicDistance(double x, double y, double &distance);
    bool calcDistanceToOneCar(double x,double y,int car_num,double &distance);
	bool calcDistanceToStaticObj(double x,double y,int obj_num,double &distance);
private:
	bool calcAttractionForce(int layer, int layer_num);
	bool calcRepusionForce(int layer, int layer_num);
	bool calcContractionForce(int layer_num1, int layer2, int layer_num2 ,int layer_num3, pair_d &force);
};