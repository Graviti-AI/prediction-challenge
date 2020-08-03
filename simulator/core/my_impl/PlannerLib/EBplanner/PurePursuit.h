#pragma once

#include<vector>
#include"EdgeAugumented_Planner.h"
#include"SpatialLattice.h"
#include "string.h"
#include <string>
#ifndef _PLANNER_STRUCT_
#define _PLANNER_STRUCT_

struct GoalPoint {
	double _x;
	double _y;
	double _theta;
	double flag;
	GoalPoint(void) {};
	GoalPoint(int x, int y, int theta,int f) {
		_x = x;
		_y = y;
		_theta = theta;
		flag = f;
	}
};



class EBPurePursuit
{
public:
	vector<GoalPoint> 	goal_point;
	double			 	RatioToReal = RATIO_TO_REAL;  //the pixel axis 's ratio to the real axis
	double            	LookAheadDistance = LOOK_AHEAD_DISTANCE;
	double            	Speed = SPEED;
	double 				distanceWithinGoal = DISTANCE_WITHIN_GOAL;
	double            	LengthOfBicycleModel = LENGTH_OF_BICYCLE_MODEL ;
	double              control_period = TIME_RESOLUTION;

	pathPoint		  	CurrentPoint;//used by bicycle model smoother

	vector<pathPoint>	real_path;
	vector<pathPoint>   DownSampledPath;

	double find_min_amongThreeA(double a, double b, double c);

	//vector<vector<costOFone>>   TrajectoryBucketSearch5;
	void FindCurrentLocation();   //determine the current location of the vehicle 
	void CalcCurvature();
	void getLookaheadPoint(pair_d &lookahead);
	void CalcBicycleModel(pair_d lookahead);
	bool ComputeKCurve(int start_point, double& a, double& b, double& c, double& d);
    double signum(double n);
private:
	void calcArcLength();
    //void FollowTrajectory(double goal_x,double goal_y, double x_now, double y_now , double theta_now);
};
#endif