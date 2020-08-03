#pragma once
#include "Vehicle.h"
#include <cmath>
#include "Environment.h"

class Bodydynamics:public Vehicle
{
public:
	struct motionstate
	{
		double x, y;//coordinates of the vehicle's mass center
		double v;
		double a, a_side;
		double Yaw,dYawdt=0, phi;
		//dYawdt:Yaw rate;phi:the angle between the direction of velocity and the vehicle body
	};
	motionstate tmpmotion,premotion;
	struct posestate
	{
		double zm=0.0,zm_dot=0.0;
		double pitchAng=0.0;
		double rollAng=0.0;
		double pitchAng_dot=0.0, rollAng_dot=0.0;
	};
	double z_sus[4] = { 0.0 };
	double zdot_sus[4] = { 0.0 };
	double v_wheel[4][2] = {{ 0.0 }};
	//0-fl,1-fr,2-rr,3-rl
	posestate prepose, tmppose, nxtpose;
	Environment Env1;
	
	double F[4][3] = {{0.0}};//the force given by the  suspension
	double Fx = 0.0, Fy = 0.0; //the force give by the environment;
	void Bodyinit(double x_, double y_, double vx_, double vy_, double Yaw_, double dYawdt_);
	void motionupdate(double dt);
	void poseupdate(double dt);
	void Outputz();
	void Calv_w();
};

