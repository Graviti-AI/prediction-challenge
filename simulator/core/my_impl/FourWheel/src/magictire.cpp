#include <cmath>
#include "magictire.h"
#include "suspension.h"
#include "singletire.h"

#include <iostream>

double magicFx(double Fztire, double slipratio) {
	double Fx, b0, b1, b2, b3, b4, b5, b6, b7, b8;
	b0 = 1.65;
	b1 = -21.3;
	b2 = 1144;
	b3 = 49.6;
	b4 = 226;
	b5 = 0.069;
	b6 = -0.006;
	b7 = 0.056; 
	b8 = 0.486;

	

	double slipr,Fz;
	slipr = 100 * slipratio;
	Fz = Fztire / 1000;
	double Dx,  Bx, Ex, BCDx;
	if (Fz <=0)
	{
		Fx=0;
	}
	else
	{
	//std::cout << "Fz: " << Fz << std::endl;
	
	Dx = b1 * pow(Fz, 2) + b2 * Fz;
	BCDx = (b3*pow(Fz, 2) + b4 * Fz) * exp(-b5 * Fz);

	Bx = BCDx / (b0*Dx);
	Ex = b6 * pow(Fz, 2) + b7 * Fz + b8;
	Fx = Dx * sin(b0 * atan(Bx *slipr - Ex * (Bx*slipr - atan(Bx*slipr))));
	}/*
	if (abs(slipratio) < 0.0001)
	{
		Fx = 0;
	}
	else Fx = Fz * abs(slipratio) / slipratio * 0.7;*/
	return Fx;
}

double magicFy(double Fztire, double slipangle) {
	double rollangle = 0;

	double Fy, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12;
	a0 = 1.3;
	a1 = -22.1;
	a2 = 1011;
	a3 = 1078;
	a4 = 1.82;
	a5 = 0.208;
	a6 = 0;
	a7 = -0.354;
	a8 = 0.707;
	a9 = 0.028;
	a10 = 0;
	a11 = 14.8;
	a12 = 0.022;

	double slipa, Fz;
	slipa = 180 * slipangle / 3.1415926;
	Fz = Fztire / 1000;
	double Dy,  By, Ey, BCDy, Sh, Sv, x;
	if (Fz<=0)
	{
		Fy=0;
	}
	else{
	Dy = a1 * pow(Fz, 2) + a2 * Fz;
	BCDy = a3 * sin(a4*atan(a5*Fz))*(1 - a12 * std::abs(rollangle));
	By = BCDy / (a0*Dy);
	Ey = a6 * pow(Fz, 2) + a7 * Fz + a8;
	Sh = a9 * rollangle;
	Sv = (a10*pow(Fz, 2) + a11 * Fz)*rollangle;
	x = Sh + slipa;
	Fy = Dy * sin(a0 * atan(By *x - Ey * (By*x - atan(By*x)))) + Sv;
	}
	return Fy;
}