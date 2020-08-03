#pragma once
#include "Vehicle.h"
#include <cmath>

class steering: public Vehicle
{
public:
	double Ratio;
	double Ang_fl=0, Ang_fr=0;
	double Ang=0;
	void steerInit(double Ratio_);
	void Ackermansteering(double steeringAng);
};

