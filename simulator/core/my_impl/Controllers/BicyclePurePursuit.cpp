#include "BicyclePurePursuit.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

void Bicycle_PurePursuit::stateInitial(double x0, double y0, double yaw0, double v0)
{
	x = x0;
	y = y0;
	yaw = yaw0;
	v = v0;
}

void Bicycle_PurePursuit::stateUpdate()
{
	x += v * cos(yaw + b)*dt;
	y += v * sin(yaw + b)*dt;
	yaw += (v / lr)*sin(b)*dt;
}

void Bicycle_PurePursuit::pursuit_a(double xref, double yref)
{
	double dist = (xref - x)*(xref - x) + (yref - y)*(yref - y);
	double vref = sqrt(dist) / dt;
	a = (vref - v) / dt;
	if (a > 1.0)a = 1.0;
	if (a < -1.0)a = -1.0;
	v += a * dt;
}

void Bicycle_PurePursuit::pursuit_df(double xref, double yref)
{
	b = atan((yref - y) / (xref - x)) - yaw;
	df = atan(tan(b)*L / lr);
	if (df > M_PI / 6.0)df = M_PI / 6.0;
	if (df < -M_PI / 6.0)df = -M_PI / 6.0;
	b = atan((lr / L)*tan(df));
}