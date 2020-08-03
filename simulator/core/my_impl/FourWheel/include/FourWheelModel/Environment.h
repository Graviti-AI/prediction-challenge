#ifndef FOURWHEEL_ENVIRONMENT_H
#define FOURWHEEL_ENVIRONMENT_H

class Environment
{
private:

public:
	double rho_air = 1.16;//the density of the air
	double gravity = 9.8;
	double f = 0.01;//the rolling resistance
	double u = 0.7;//adhesive rate
	double i_slope1=0,i_slope2=0;//slope
	double wind_x=0;
	double wind_y=0;
	void setwind();

};

#endif