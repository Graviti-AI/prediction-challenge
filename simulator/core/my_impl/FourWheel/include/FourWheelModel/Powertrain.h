#pragma once
#include "Vehicle.h"
#include "Bodydynamics.h"
#define PI 3.1415926536 
class Powertrain :public Bodydynamics
{
private:
	int k=2;//indicating the gear
	double ig;
public:
	bool clutch;
	double i[8] = { -3,0,3,2.4,2,1.6,1.3,0.9 };
	double n;//The engine speed     r/min
	double Torq;
	double throttle;

	void autotrans();
	void EngineTorq();
};

