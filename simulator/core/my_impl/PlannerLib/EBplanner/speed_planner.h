#pragma once

#include<vector>
#include"EdgeAugumented_Planner.h"
#include"SpatialLattice.h"
#include "string.h"
#include <string>
#include "Global_parameter.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

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

#endif

struct Cost
{
	double moving_obstacles_cost = 0;
	double lateral_acceleration_cost = 0;
	double longitudinal_acceleration_cost = 0;
	double jerk_cost = 0;
	double temporal_diff_from_ref_cost = 0;

};

struct parameter_a {
	double p0;
	double p1;
	double p2;
	double speed_t1;
    double stop_time;
	double pass_possibility_;
};


struct StopLine{
    double x0;
    double y0;
    double x1;
    double y1;
    double distance;
    double existance;
};

struct ObstacleTrajectory
{
	double radius = sqrt(pow(0.5 * WIDTH_OF_CAR, 2) + pow(LENGTH_OF_CAR / 6, 2));
	vector<pathPoint> trajectory;
};


class SpeedPlanner
{
public:
	vector<GoalPoint> goal_point;
	double			 WidthOfCar = WIDTH_OF_CAR ;
	double           LengthOfCar = LENGTH_OF_CAR ;
	double           radius_car; //use three circles to approximate our car

    double           current_speed;
	double			 MaxLateralAcceleration = MAX_LATERAL_ACCELERATION;
	double			 MaxLongitudinalDeceleration = MAX_LONGITUDINAL_DECELERATION;
	double			 MaxLongitudinalAcceleration = MAX_LONGITUDINAL_ACCELERATION;
	double			 MaxSpeed = MAX_SPEED;
	double           jerk_max = MAX_JERK ;

    double           control_period = 1 / SPEED_PLANNER_FREQUENCY;  //control circle  0.1s, should be 1/frequency , which is planner frequency

	double           dx = 558700 - 63;
	double           dy = 4196800 - 180;
	double           dth = - 1.8366 - 0.9 * 0.3491;
	vector<pathPoint>	real_path;

	void SpeedPlanByConstraints();
    virtual bool PlanSpeed(int end_point) = 0;
protected:
    inline double ComputeTwoPointDistance(double x1, double y1, double x2, double y2)
	{
    	return pow(x1 - x2, 2) + pow(y1 - y2, 2);
	}
	inline double TransformToOriginalCoor(double &x, double &y, double &theta)
	{
		double temp_x = x - dx;
		double temp_y = y - dy;
		theta -= dth;
		x = temp_x * cos(-dth) - temp_y * sin(-dth);
		y = temp_x * sin(-dth) + temp_y * cos(-dth);

	}
	inline double TransformToOriginalCoor(double &x, double &y)
	{
		// counter clockwise
		double temp_x = x - dx;
		double temp_y = y - dy;
		x = temp_x * cos(-dth) - temp_y * sin(-dth);
		y = temp_x * sin(-dth) + temp_y * cos(-dth);
	}
	// inline Eigen::MatrixXd TransformToOriginalCoor(Eigen::MatrixXd &matrix)
	// {	cout << "transform 1" << endl;
	// 	Eigen::MatrixXd temp_x = matrix.row(0);
	// 	Eigen::MatrixXd temp_y = matrix.row(1);
	// 	Eigen::MatrixXd ans_x = matrix.block(0,0,matrix.rows(),1);
	// 	Eigen::MatrixXd ans_y = matrix.block(0,1,matrix.rows(),1);
	// 	cout << "transform 2" << endl;
	// 	temp_x = temp_x.array() - dx;
	// 	temp_y = temp_y.array() - dy;
	// 	ans_x = temp_x.array() * cos(-dth) - temp_y.array() * sin(-dth);
	// 	ans_x = temp_x.array() * sin(-dth) + temp_y.array() * cos(-dth);
	// 	cout << "transform 3" << endl;
	// 	cout << "ans_x" << endl << ans_x << endl;
	// 	cout << "ans_y" << endl << ans_y << endl;
	// 	matrix.col(0) = ans_x.col(0);
	// 	matrix.col(1) = ans_y.col(0);
	// }
};
