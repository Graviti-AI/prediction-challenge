#pragma once
#include <iostream>

class Bicycle_PurePursuit
{
private:
	double lr = 1.5;
	double L = 2.6;
	double dt = 0.1;
	double a;
	double df;
	double b;

public:
	double x;
	double y;
	double yaw;
	double v;
	void pursuit_a(double xref, double yref);
	void pursuit_df(double xref, double yref);
	void stateInitial(double x0, double y0, double yaw0, double v0);
	void stateUpdate();
};