#pragma once
#include "suspension.h"
#include "magictire.h"
#include "steering.h"

class singletire : public susp, public Vehicle
{
public:
	double r; // the radius of wheel
	double mtire;// the unsprung mass
	double Iw;//  rotational inertia of wheel


	struct tirepara
	{
		double strAng;
		double slipratio;// the ratio of tire 
		double slipangle;//the slipangle of tire
		double Fytire;//tire force, in the latitude direction
		double Fxtire;//tire force, in the longitude direction
		double Fztire;//tire force, in the direction vertical to ground
		double Ztire;//the height of wheel
		double vXtire;//the longitude speed of tire
		double vYtire;//the latitude speed of tire
		double rolltire;//the rotate speed of tire rolling
	};
	tirepara tire, tirepre;

	void tireini(double n,double r, double mtire, double Iw,bool front);
	void tirestate(double T_shaft,double M, double If, double Tf, double dt, double i,bool front);
	void Foutput(double T_shaft, double M, bool front);
};