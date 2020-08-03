#pragma once
#include <iostream>
#include <vector>
#define PI (3.14159265358979323846)

typedef std::vector<double> Vector;

class PidController
{
private:
	double s_pre=0;
	double u_frenet = 0;
	double s_frenet = 0;
	double S_integral = 0;
	double theta1=0;
	double theta2=0;
	double l=0;
	double a = 0, b = 0;
public:
	PidController(double tspan);
	~PidController();

	//CAR STATE
	double x = 0;
	double y = 0;
	double t_delta=0;
	double x_nxt = 0;
	double y_nxt = 0;
	double Yaw = 0;
	double v = 0;
	double phi = 0;
	double dYawdt = 0;
	double tspan;
	//CAR INPUT
	double output_theta;//setting theta in [-PI/4,PI/4], but it mus be changed continuously
	double output_a;//setting a in [-5,5]
	
	
	int count=0;
	

	std::vector<double> t;
	std::vector<double> x_map;
	std::vector<double>	y_map;
	std::vector<double> v_plan;
	bool finish();
	double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
	void locupdate_temporary();
	void locupdate_prediction();
	
	void update(double state[6]);
	void PIDcontrol(double tspan);

	Vector internalUpdate(const Vector &currState, const Vector &planResult);
};

