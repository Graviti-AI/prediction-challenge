#pragma once
class Transmission
{
private:
	int k = 2;//indicating the gear
	double i[8] = { -3.0,0.0,3.5,2.0,1.4,1.0,0.7,0.58 };
public:
	bool clutch=true;
	double n;//The engine speed     r/min
	double i_;
	void transinit();
	void autotrans();
};

