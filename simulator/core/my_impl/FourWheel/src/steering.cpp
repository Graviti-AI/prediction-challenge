#include "steering.h"
#include <iostream>
void steering::steerInit(double Ratio_)
{
	Ratio = Ratio_;
	Ang = 0;
	this->Vehiclesetting();
};
void steering::Ackermansteering(double steeringAng)
{
	double l = lf + lr;

	Ang = steeringAng / (Ratio);
	double tanAng = tan(Ang);
	if (Ang==0)
	{
		Ang_fl = 0;
		Ang_fr = 0;
	}
	else
	{
		//std::cout << l << "    " << d << "    " << steeringAng << "    " << Ratio << std::endl;
		Ang_fl = atan(2.0 * l*tanAng / (2.0 * l - d * tanAng));
		Ang_fr = atan(2.0 * l*tanAng / (2.0 * l + d * tanAng));
	}
};
