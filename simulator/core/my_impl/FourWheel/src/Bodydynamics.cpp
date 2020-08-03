#include <iostream>
#include "Bodydynamics.h"
using namespace std;
void Bodydynamics::Bodyinit(double x_, double y_, double vx_, double vy_, double Yaw_, double dYawdt_)
{
	this->Vehiclesetting();
	tmpmotion.x = x_;
	tmpmotion.y = y_;
	tmpmotion.Yaw = Yaw_;
	tmpmotion.dYawdt = dYawdt_;
	tmpmotion.v = pow((pow(vx_, 2.0) + pow(vy_, 2.0)), 0.5);
	if (tmpmotion.v == 0)
		tmpmotion.phi = 0;
	else if (vx_ == 0)
		tmpmotion.phi = 3.14159265358 / 2.0 - Yaw_;
	else if (vy_ == 0)
		tmpmotion.phi = -Yaw_;
	else
		tmpmotion.phi = atan2(vy_, vx_) - Yaw_;

	tmpmotion.a = 0;
	tmpmotion.a_side = 0;
	//
}
void Bodydynamics::motionupdate(double dt)
{
	if (tmpmotion.v > 0.3)
	{
		Fx = abs((Env1.wind_x - tmpmotion.v*cos(tmpmotion.phi))) / (Env1.wind_x - tmpmotion.v*cos(tmpmotion.phi))* A1 * Env1.rho_air*pow((Env1.wind_x - tmpmotion.v*cos(tmpmotion.phi)), 2);

	}//Fy = - A2 * Env1.rho_air*pow((-tmpmotion.v*sin(tmpmotion.phi)), 2);
	else
	{
		Fx = 0;
	}
	
	if (isnan(Fx)) cout << "xxxxx: 1" << endl;

	premotion = tmpmotion;
	tmpmotion.v += premotion.a*dt;
	if (tmpmotion.v<0) {
		tmpmotion.v = 0;
	}
	//cout << "F[0][0] " << F[0][0] << " F[1][0] " << F[1][0] << " F[2][0] " << F[2][0] << " F[3][0] " << F[3][0] << endl;
	tmpmotion.a = (F[0][0] + F[1][0] + F[2][0] + F[3][0] + Fx)*cos(premotion.phi) / m + (F[0][1] + F[1][1] + F[2][1] + F[3][1] + Fy)*sin(premotion.phi) / m;
	//cout << "premotion.phi" << premotion.phi << endl;
	//cout << "F[0][0] + F[1][0] + F[2][0] + F[3][0]  " << F[0][0] + F[1][0] + F[2][0] + F[3][0] << endl;
	//cout << "F[0][1] + F[1][1] + F[2][1] + F[3][1] + Fy" << F[0][1] + F[1][1] + F[2][1] + F[3][1] + Fy << endl;
	tmpmotion.a_side = (F[0][1] + F[1][1] + F[2][1] + F[3][1] + Fy)*cos(premotion.phi) / m - (F[0][0] + F[1][0] + F[2][0] + F[3][0] + Fx)*sin(premotion.phi) / m;
	if (isnan(tmpmotion.v)) cout << "xxxxx: 2" << endl;
	if (isnan(tmpmotion.a_side)) cout << "xxxxx: 3" << endl;
	// cout << m << endl;
	// cout << F[0][1] << endl;
	// cout << F[1][1] << endl;
	// cout << F[2][1] << endl;
	// cout << F[3][1] << endl;
	// cout << F[0][0] << endl;
	// cout << F[1][0] << endl;
	// cout << F[2][0] << endl;
	// cout << F[3][0] << endl;
	// cout << Fx << endl;
	// cout << Fy << endl;
	// cout << premotion.phi << endl;
	if (isnan(tmpmotion.a)) cout << "xxxxx: 4" << endl;
	tmpmotion.Yaw += premotion.dYawdt*dt;

	//cout << F[0][0] << endl;
	//cout << F[1][0] << endl;
	//cout << F[2][0] << endl;
	//cout << F[3][0] << endl;
	
	
	if (abs(F[2][0] + F[1][0] - F[0][0] - F[3][0]) < 0)
	{

		//cout << "F[2][0] + F[1][0] - F[0][0] - F[3][0]  " << F[2][0] + F[1][0] - F[0][0] - F[3][0] << endl;
		//		tmpmotion.dYawdt += ((0.0)*d / 2.0 + (F[0][1] + F[1][1])*lf - (F[2][1] + F[3][1])*lr) / Iz;
	}
	else
	{
		//cout << "F[0][1] + F[1][1]" << F[0][1] + F[1][1] << endl;
		//cout << "F[2][1] + F[3][1]" << F[2][1] + F[3][1] << endl;

		tmpmotion.dYawdt += dt * ((F[2][0] + F[1][0] - F[0][0] - F[3][0])*d / 2.0 + (F[0][1] + F[1][1])*lf - (F[2][1] + F[3][1])*lr) / Iz;
	}
	//if (abs(tmpmotion.dYawdt) < 0.05)
	//{
		//tmpmotion.dYawdt = 0;
	//}
	if (premotion.v > 5e-1)
		tmpmotion.phi += (premotion.a_side*dt) / premotion.v;
	else
		tmpmotion.phi = 0.0;

	tmpmotion.x += (premotion.v + tmpmotion.v)/2.0* dt*cos(tmpmotion.Yaw + tmpmotion.phi);
	tmpmotion.y += (premotion.v + tmpmotion.v)/2.0* dt*sin(tmpmotion.Yaw + tmpmotion.phi);

};

void Bodydynamics::poseupdate(double dt)
{
	prepose = tmppose;
	tmppose.pitchAng += prepose.pitchAng_dot * dt;
	tmppose.rollAng += prepose.rollAng_dot*dt;

	tmppose.zm += prepose.zm_dot*dt;
	tmppose.pitchAng_dot += ((F[2][2] + F[3][2])*lr - (F[0][2] + F[1][2])*lf- m * premotion.a*cos(premotion.phi)*h+ m * premotion.a_side*sin(premotion.phi)*h) *dt/ Ix;
	//cout << "(F[0][2] + F[3][2] - F[1][2] - F[2][2]) " << (F[0][2] + F[3][2] - F[1][2] - F[2][2]) << endl;
	//cout << "premotion.a " << premotion.a << endl;
	//cout << "premotion.a_side " << premotion.a_side << endl;

	tmppose.rollAng_dot += ((F[0][2] + F[3][2] - F[1][2] - F[2][2])*d/2.0-m* premotion.a * sin(premotion.phi)*h + m* premotion.a_side*cos(premotion.phi)*h) *dt/ Iy;
	//if (abs(tmppose.rollAng_dot) < 0.0001)
	//{
	//	tmppose.rollAng_dot = 0;
	//}
	//cout << "tmppose.rollAng_dot" << tmppose.rollAng_dot << endl;
	tmppose.zm_dot += (F[0][2] + F[2][2] + F[3][2] + F[1][2] - m * Env1.gravity)*dt / m;
	//std::cout << "zm_dot   " << tmppose.zm_dot << std::endl;
	/*
	if (abs(tmppose.pitchAng) > 0.005)
	{
		tmppose.pitchAng = tmppose.pitchAng / abs(tmppose.pitchAng)*0.005;
		tmppose.pitchAng_dot = 0;
	}
	if (abs(tmppose.rollAng) > 0.005)
	{
		tmppose.rollAng = tmppose.rollAng / abs(tmppose.rollAng)*0.005;
		tmppose.rollAng_dot = 0;
	}

	if (abs(tmppose.zm) > 0.05)
	{
		tmppose.zm = tmppose.zm / abs(tmppose.zm)*0.05;
		tmppose.zm_dot = 0;
	}*/
};
void Bodydynamics::Outputz()
{
	z_sus[0] = tmppose.zm - tmppose.pitchAng*lf + tmppose.rollAng*d / 2.0;//fl
	zdot_sus[0] = tmppose.zm_dot - tmppose.pitchAng_dot*lf + tmppose.rollAng_dot*d / 2.0;
	z_sus[1] = tmppose.zm - tmppose.pitchAng*lf - tmppose.rollAng*d / 2.0;//fr
	zdot_sus[1] = tmppose.zm_dot - tmppose.pitchAng_dot*lf - tmppose.rollAng_dot*d / 2.0;
	z_sus[2] = tmppose.zm + tmppose.pitchAng*lr - tmppose.rollAng*d / 2.0;//rr
	zdot_sus[2] = tmppose.zm_dot + tmppose.pitchAng_dot*lr - tmppose.rollAng_dot*d / 2.0;
	z_sus[3] = tmppose.zm + tmppose.pitchAng*lr + tmppose.rollAng*d / 2.0;//rl
	zdot_sus[3] = tmppose.zm_dot + tmppose.pitchAng_dot*lr + tmppose.rollAng_dot*d / 2.0;
};
void Bodydynamics::Calv_w()
{
	//cout <<"dYawdt "<< tmpmotion.dYawdt << endl;
	if (abs(tmpmotion.dYawdt) > 0)
	{
		v_wheel[0][0] = tmpmotion.v*cos(tmpmotion.phi) - d / 2.0 * tmpmotion.dYawdt;
		v_wheel[0][1] = tmpmotion.v*sin(tmpmotion.phi) + lf * tmpmotion.dYawdt;
		v_wheel[1][0] = tmpmotion.v*cos(tmpmotion.phi) + d / 2.0 * tmpmotion.dYawdt;
		v_wheel[1][1] = tmpmotion.v*sin(tmpmotion.phi) + lf * tmpmotion.dYawdt;
		v_wheel[2][0] = tmpmotion.v*cos(tmpmotion.phi) + d / 2.0 * tmpmotion.dYawdt;
		v_wheel[2][1] = tmpmotion.v*sin(tmpmotion.phi) - lr * tmpmotion.dYawdt;
		v_wheel[3][0] = tmpmotion.v*cos(tmpmotion.phi) - d / 2.0 * tmpmotion.dYawdt;
		v_wheel[3][1] = tmpmotion.v*sin(tmpmotion.phi) - lr * tmpmotion.dYawdt;
	}

	else
	{
		v_wheel[0][0] = tmpmotion.v*cos(tmpmotion.phi) ;
		v_wheel[0][1] = tmpmotion.v*sin(tmpmotion.phi);
		v_wheel[1][0] = tmpmotion.v*cos(tmpmotion.phi) ;
		v_wheel[1][1] = tmpmotion.v*sin(tmpmotion.phi) ;
		v_wheel[2][0] = tmpmotion.v*cos(tmpmotion.phi) ;
		v_wheel[2][1] = tmpmotion.v*sin(tmpmotion.phi) ;
		v_wheel[3][0] = tmpmotion.v*cos(tmpmotion.phi) ;
		v_wheel[3][1] = tmpmotion.v*sin(tmpmotion.phi) ;

	}
};


