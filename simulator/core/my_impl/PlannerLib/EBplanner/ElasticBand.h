#pragma once
#include<vector>
#include <cmath>
#include "Global_parameter.h"
using namespace std;

typedef pair<int, int> pair_i;
typedef pair<double, double> pair_d;

struct force_ {
	double _x;
	double _y;
	force_(void) {};
	force_(pair_d p) {
		_x = p.first;
		_y = p.second;
	}
};

//-------------------Define a Force------------
//an artifical force has two types: an internal contraction force and an external repulsion force
struct Force {
	force_ _contraction;
	force_ _repulsion;
	force_ _attraction;
	Force(void) { };
	Force(pair_d c, pair_d r, pair_d a) {
		_contraction = force_(c);
		_repulsion = force_(r);
		_attraction = force_(a);
	}
};

struct Torque {
	double _contraction;
	double _repulsion;
	double _attraction;
	Torque(void) {};
};

//----------------Define a Bubble---------------------
//a Bubble has: (1)pose(x,y) (2)magnitude(r) (3)artifical force 
struct Bubble {
	double _x;
	double _y;
	double _r;
	Force force;
	Bubble(void) {};
	Bubble(pair_d p, double r) {
		_x = p.first;
		_y = p.second;
		_r = r;
		force = Force();
	}
	Bubble(pair_d p, double r, pair_d c, pair_d rr, pair_d a) {
		_x = p.first;
		_y = p.second;
		_r = r;
		force = Force(c, rr, a);
	}
};

//----------------Define a car_Bubble-----------------
//----------------simplified by three circles---------
struct Car_Bubble {
	double _x1;    //the middle circle's center location
	double _y1;
	double _costheta;
	double _sintheta;
	double _theta;
	double _r;
	double _x0;
	double _y0;
	double _x2;
	double _y2;;
	Force  force;
	Torque torque;
	Car_Bubble(void) {}
	Car_Bubble(pair_d point1, double theta, double r)
	{
		_x1 = point1.first;
		_y1 = point1.second;
		_theta = theta;
		_r = r;
		_x2 = _x1 + r * cos(theta);
		_y2 = _y1 + r * sin(theta);
		_x0 = _x1 - r * cos(theta);
		_y0 = _y1 - r * sin(theta);
		force = Force();
		torque = Torque();
	}
};

//----------------Define a Obstacle-------------------
//consider an easy situation--simplify a obstacle as a circle
//---------------NEED TO BE MODIFIED------------------
struct Obstacle {
	double _x;
	double _y;
	double _r;
	Obstacle(void) {}
	Obstacle(pair_d p, double r) {
		_x = p.first;
		_y = p.second;
		_r = r;
	}
};

class ElasticBand {
public:

	vector<pair_i>           _fullelastic;
	vector<Bubble>			_band;
	vector<Obstacle> 	    _obstacle;
	vector<Force>            _force;
	vector<Car_Bubble>		_carBand;
	int                      _initradius;
	int						num_optim_iterations_ = NUM_OPTIM_ITERATIONS_;  //the number of iterations when doing optim
	int						BubbleNum;  //the number of bubbles
	double					centerline_y = CENTERLINE_Y;//the y location of centerline-->used to compute attract forces
	double                  tiny_bubble_distance_;//mim
	double					car_length = CAR_LENGTH;
	double					car_width = CAR_WIDTH;
	double					tiny_bubble_expansion = TINY_BUUBLE_EXPANSION; //lower bound for bubble expansion. below this bound bubble is considered as "in collision"
	double                  attract_force_gain = ATTRACT_FORCE_GAIN;
	double                  internal_force_gain = INTERNAL_FORCE_GAIN;
	double                  external_force_gain = EXTERNAL_FORCE_GAIN;
	double					max_bubble_radius = MAX_BUBBLE_RADIUS;
	double					min_bubble_overlap = MIN_BUBBLE_OVERLAP;
	double					StepSize_ = STEPSIZE_;  //the step size of [the min distance form bubble to obstacle]
	void initializeElastic(pair_d start, pair_d end, int nparticles, int ir);
	void deformElastic();
private:
	void getAllParticles(vector<pair_i> &elasticv, pair_i point1, pair_i point2);
	bool isBubbleIntersect(Bubble b1, Bubble b2);
	void respondToobstacles(int bubble_num);
	bool calcAttractForces(int bubble_num);
	bool calcInternalForces(int bubble_num);
	bool calcExternalForces(int bubble_num);
	bool calcObstacleKinematicDistance(pair_d point, double &distance);
	bool calcMinDistance(int bubble_num);//compute the radius of bubbles
	bool refineBand();
	bool removeAndFill(vector<Car_Bubble>& band, vector<Car_Bubble>::iterator& start_iter, vector<Car_Bubble>::iterator& end_iter);
	bool fillGap(vector<Car_Bubble>& band, vector<Car_Bubble>::iterator& start_iter, vector<Car_Bubble>::iterator& end_iter);
	bool interpolateBubbles(pair_d start_point, pair_d end_point, pair_d& interpolated_center, pair_d& interpolated_theta);
	bool isCarBubbleIntersect(Car_Bubble b1, Car_Bubble b2);
};

