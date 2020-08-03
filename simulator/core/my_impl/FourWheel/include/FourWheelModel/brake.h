#pragma once
#include "Vehicle.h"
#include "singletire.h"
#include <cmath>
class brake:public Vehicle
{
private:
	double Kb;//on each wheel, the braking moment M=Kb*pressure;
public:
	double pressure[4] = {0,0,0,0};
	double pressurepre[4] = {0,0,0,0};
	double n[4];
	double v_w[4];
	double M[4] = {0,0,0,0};
	double rw;
	double slip[4];
	double brakepedal;//the depth of brakepedal controls the braking pressure
	void brakeInit(double Kb);
	void ABScontrol();
	void ESPcontrol(double theta);//R is the radius of the turning circle, R<R0 meaning that the car is oversteer.theta=R/R0;
	void brakeInput(double brake_);
	void Output();
};

