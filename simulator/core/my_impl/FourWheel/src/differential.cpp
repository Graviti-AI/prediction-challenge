// #include<iostream>
#include "differential.h"
#include <cmath>
//initialize the differental character and state
void diff::diffini()
{
	ig = 4.1; //gear ratio of differential 
	k1 = -90.0 / 60.0; // the factor to describe the functional relation between Wheel Speed difference and the Torque difference
	k2 = -90.0 / 180.0;
	k3 = -90.0 / 300.0;
	k4 = -90.0 / 420.0; 
	diffst.Tin = 0; // the torque from transmission
	diffst.Tl = 0; // the torque to the left wheel
	diffst.Tr = 0; // te torque to the right wheel
};

//input the previous state of wheel rotate speed and torque from transmission
void diff::diffstate(double prerolltire_r, double prerolltire_l, double preTrans)
{
	diffstpre = diffst;
	diffst.Tin = preTrans*ig;
	double rolltirediff = prerolltire_r - prerolltire_l;
	double Tdiff = 0;
	//if (abs(rolltirediff) > 700)
		//rolltirediff = rolltirediff / abs(rolltirediff) * 700.0;
	if (std::abs(rolltirediff) <= 60)
		Tdiff = k1 * std::abs(rolltirediff);
	else
	{
		if (std::abs(rolltirediff) <= 240)
			Tdiff = -90 + k2 * (std::abs(rolltirediff) - 60);
		else {
			if (std::abs(rolltirediff) <= 540)
				Tdiff = -180 + k3 * (std::abs(rolltirediff) - 240);
			else
				Tdiff = -270 + k4 * (std::abs(rolltirediff) - 540);
		}
	}

	if (rolltirediff < 0)
	{
		Tdiff = -Tdiff;
	}

	diffst.Tl = (diffst.Tin / 2 - Tdiff);
	diffst.Tr = (diffst.Tin / 2 + Tdiff);
}