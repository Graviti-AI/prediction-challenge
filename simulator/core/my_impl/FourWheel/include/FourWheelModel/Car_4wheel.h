#pragma once
#include "Bodydynamics.h"
#include "brake.h"
#include "differential.h"
#include "Engine.h"
#include "singletire.h"
#include "suspension.h"
#include "Transmission.h"
#include "Environment.h"
#include "magictire.h"
#include "steering.h"
#include <ctime>
#define PI (3.1415926536)
/*
create a class car_4wheel   
call the function "Carinit" to initialize the speed,location of the car
call the function "getinput" to input the throttle pedal,brake pedal and steedingwheel angle of the car,the sequence of the input is shown in the function;
call the function "Carupdate" to update the car's state, including the x,y,vx,vy,Yaw,dYawdt of the car.
You can get the latest state from the public array output.
*/

#include "../../../others.hpp"

const double dt = SIM_TICK;

class Car_4wheel: public Vehicle
{
protected:
	
public:

	double x_temp=0, y_temp=0;
	double v_w[4][2] = {{ 0 }};
	double n = 800;
	double n_ = 0;
	double n_w[4] = { 0 };
	double z[4] = { 0 };
	double strAng[4] = { 0 };
	double z_dot[4] = { 0 };
	double F_sus[4][3] = {{0}};
	double T_shaft[4] = {0};
	double M_brake[4] = {0};
	double i_car=3;
	double output[6] = {0}; 
	/*
	the sequence of the output;
	output[0] is the x coordinate;
	output[1] is the y coordinate;
	output[2] is the Yaw of the car's body;
	output[3] is the vx;
	output[4] is the vy;
	output[5] is the dYawdt;
	*/
	double R = 0;
	double theta_ = 0;
	singletire Wheel[4];
	Bodydynamics Body;
	brake Brake;
	steering Str;
	Engine Eng;
	Transmission Trans;
	diff Diff;
	double brakepedal=0;
	double throttlepedal=0;
	double strwheel=0;
	double slipratio[4] = { 0 };
	void Carinit(double x_, double y_, double vx_, double vy_, double Yaw_, double dYawdt_);
	void BodyreadF();
	void BodyOutput();
	void BodyUpdate();
	void WheelInput();
	void Wheeloutput();
	void stroutput();
	void CalR();
	void Engoutput();
	void Brakeoutput();
	void Carupdate(double time_gap);
	void coordinate_change(double x, double y, double phi);

	void getinput(double throttle_, double brake_, double strwheel_);
	//void updatetick(int argc, double argv[]);
};

