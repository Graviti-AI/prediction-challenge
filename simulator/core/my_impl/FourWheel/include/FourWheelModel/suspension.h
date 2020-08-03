#ifndef FOURWHEEL_SUSPENSION_H
#define FOURWHEEL_SUSPENSION_H
#include "Bodydynamics.h"


class susp
{
public:
	double suspK;
	double suspC;
	double Z_ground = 0, Zdot_ground = 0;
	double z = 0, z_dot = 0;
	struct susppara 
	{
		double Fzsusp=0;
		double Zsusp=0;
		double Z_dot=0;
	};
	susppara suspension, suspensionpre;
	void suspini(double suspK, double suspC);
	void suspstate();
};

#endif