#include<cmath>
#include <iostream>
#include "singletire.h"
#include "suspension.h"
#include "magictire.h"
using namespace std;

//initialize one tire
void singletire::tireini(double n,double r, double mtire, double Iw,bool front)
{
	this->Vehiclesetting();
	tire.strAng = 0;
	tire.slipangle = 0; //the slipangle of tire
	tire.slipratio = 0;  // the ratio of tire 
	tire.Fxtire = 0; //tire force, in the longitude direction
	tire.Fytire = 0; //tire force, in the latitude direction
	if (front)
		tire.Fztire = lr / (lf + lr) * 1 / 2.0*m*9.8;
	else
		tire.Fztire = lf / (lf + lr) * 1 / 2.0*m*9.8;//tire force, in the direction vertical to ground
	tire.rolltire = n/r; //the rotate speed of tire rolling
	tire.vXtire = 0; //the longitude speed of tire
	tire.vYtire = 0; //the latitude speed of tire
	singletire::r = r; // the radius of wheel
	singletire::mtire = mtire; // the unsprung mass
	singletire::Iw = Iw; //  rotational inertia of wheel
	tire.Ztire = 0;

};

//update the state of tire
void singletire::tirestate(double T_shaft,double M, double If,double Tf, double dt, double i,bool front)
{

	//std::cout << "Tf: " << Tf << std::endl;

	//if (front)
	//	cout << "I'm front wheel" << endl;

	//tire.Ztire = 0;

	//std::cout << "fwergcx" << std::endl;

	//std::cout << singletire::mtire << std::endl;

	//std::cout << susp::suspension.Fzsusp << std::endl;

	//std::cout << lr << std::endl;

	//std::cout << lf << std::endl;

	//std::cout << m << std::endl;
	tirepre = tire;

	if (front)
		tire.Fztire = singletire::mtire*9.8 + susp::suspension.Fzsusp;
	else
		tire.Fztire = singletire::mtire*9.8 + susp::suspension.Fzsusp;

	double k_;
	if (tire.rolltire > 0)
		k_ = 1.0;
	else if (tire.rolltire == 0)
		k_ = 0.0;
	else
		k_ = -1.0;

	tire.slipangle = atan2(tire.vYtire, tire.vXtire) ;

	if (front)
	{
		//cout << (front) << "	"<<If<<"	"<<pow(i, 2.0) <<"	"<< Iw<<endl;
		tire.rolltire += (T_shaft -k_*M- k_ * Tf - tire.Fxtire * r)*dt / ((front)*If * pow(i, 2.0) / 2 + Iw);

		/*
		if (abs((T_shaft -k_*M- k_ * Tf - tire.Fxtire * r)*dt / ((front)*If * pow(i, 2.0) / 2 + Iw))>1e20) {
			std::cout << "rolltire overflows" <<std::endl;
			throw "rolltire overflows";
		}*/
	}
	else
	{
		if(M>0)
			tire.rolltire += (-k_*M- k_ * Tf - tire.Fxtire * r)*dt / (Iw);
		else
			tire.rolltire = tire.vXtire / r;
	}
	
	//cout << tire.slipratio << endl;
}
void singletire::Foutput(double T_shaft,double M,bool front)
{
	//cout << "tire.rolltire*r "<<tire.rolltire*r<<endl;
	//cout << "tire.vXtire "<<tire.vXtire<<endl;
	//cout << "tire.rolltire"<<tire.rolltire <<endl;
	//cout << "rw :"<< r<<endl;
	if (tire.rolltire > 0 && tire.vXtire>0)
	{
		if (tire.rolltire * r > tire.vXtire)
			tire.slipratio = (tire.rolltire * r - tire.vXtire) / (tire.rolltire * r);
		else if (tire.rolltire * r < tire.vXtire)
			tire.slipratio = (tire.rolltire * r - tire.vXtire) / tire.vXtire;
	}
	else if (tire.rolltire > 0 && tire.vXtire <= 0)
	{
		tire.slipratio = 1;
	}
	else if (tire.rolltire <= 0 && tire.vXtire > 0)
	{
		tire.slipratio = -1;
	}
	else
	{
		if (tire.rolltire * r < tire.vXtire)
			tire.slipratio = -(tire.rolltire * r - tire.vXtire) / (tire.rolltire * r);
		else if (tire.rolltire * r > tire.vXtire)
			tire.slipratio = (-tire.rolltire * r + tire.vXtire) / tire.vXtire;
	}
	if (abs(tire.vXtire)<0.5)
		tire.Fytire = - 1000*tire.vYtire;
	else 
		tire.Fytire = - magicFy(tire.Fztire, tire.slipangle);

	tire.Fxtire = magicFx(tire.Fztire, tire.slipratio);
	if (abs(tire.rolltire) < 0.02&&abs(M)<10)
	{
		tire.Fxtire = (T_shaft ) / r;
	}
	tire.Fxtire=0.7*tire.Fxtire+0.3*tirepre.Fxtire;
	tire.Fytire=0.7*tire.Fytire+0.3*tirepre.Fytire;
}