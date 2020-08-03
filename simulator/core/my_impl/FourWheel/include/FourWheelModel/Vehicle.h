#pragma once
#include <cmath>
class Vehicle
{
public:
	double lf, lr;
	double A1,A2;//the frontal area and the side area
	double rw;//the radius of the wheel
	double d;//the width of each axle
	double m;//the mass of the vehicle
	double Iz, Ix, Iy;
	double If;//the Inertia of the flywheel
	double h;
	void Vehiclesetting();
};

