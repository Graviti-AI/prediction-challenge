#pragma once

class diff {
public:
	
	double ig;//gear ratio of differential 
	double k1;// the factor to describe the functional relation between Wheel Speed difference and the Torque difference
	double k2;
	double k3;
	double k4;

	struct Tdistri
	{
		double Tr;// te torque to the right wheel
		double Tl;// the torque to the left wheel
		double Tin;// the torque from transmission
	}diffst, diffstpre;

	void diffini();
	//input the previous state of wheel totate speed and torque from transmission
	void diffstate(double prerolltirer, double prerolltirel, double preTengine);

};