#include "brake.h"

#include <cmath>

void brake::brakeInit(double Kb_)
{
	Kb = Kb_;
	brakepedal = 0;
}
void brake::ABScontrol()
{
	int i = 0;
	for (i = 0; i < 4; i++)
	{
		if (std::abs(slip[i]) > 0.15)
		{
			pressure[i] = pressurepre[i]- 0.075;
			if (pressure[i]<0)
			{
				pressure[i] = 0;
			}
		}
	}
};
void brake::ESPcontrol(double theta)
{
	if (std::abs(theta) < 1)
	{
		if (theta > 0)
		{
			pressure[0] = 0.0;
			pressure[3] = 0.0;
			pressure[2] = 0.2;
			pressure[1] = 0.2;
		}
		if (theta < 0)
		{
			pressure[2] = 0.0;
			pressure[1] = 0.0;
			pressure[0] = 0.2;
			pressure[3] = 0.2;
		}
	}
};
void brake::brakeInput(double brake_)
{
	brakepedal = brake_;
	int i = 0;
	for (i = 0; i < 4; i++)
	{
		pressure[i] = pressurepre[i]+0.05;
		if (pressure[i] > brakepedal)
			pressure[i] = brakepedal;
	}
};
void brake::Output()
{
	int i;
	for (i = 0; i < 4; i++)
	{
		M[i] = pressure[i] * Kb;
		pressurepre[i] = pressure[i];
	}
};
