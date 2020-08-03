

#include "Car_4wheel.h"
#include <iostream>

#include <fenv.h>
//double dt = 0.1;
using namespace std;
void Car_4wheel::coordinate_change(double x,double y, double phi)
{
	x_temp = y * sin(phi) + x * cos(phi);
	y_temp = y * cos(phi) - x * sin(phi);
};
void Car_4wheel::Carinit(double x_, double y_, double vx_, double vy_, double Yaw_, double dYawdt_)
{
	Vehiclesetting();
	Diff.diffini();
	Body.Vehiclesetting();
	Body.Env1.setwind();
	Trans.transinit();
	
	int i;
	for (i = 0; i < 4; i++)
	{
		Wheel[i].Vehiclesetting();
		Wheel[i].tireini(n_w[i],rw,15.0,0.5,1-i/2);
		Wheel[i].suspini(17000,2000);
	}
	Brake.brakeInit(1200);
	Str.steerInit(6);
	Body.Bodyinit(x_,y_,vx_,vy_,Yaw_,dYawdt_);
};
void Car_4wheel::BodyreadF()
{

	#ifndef NDEBUG
	feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
	#endif

	int i;
	for (i = 0; i < 4; i++)
	{
		coordinate_change(F_sus[i][0], F_sus[i][1], -strAng[i]);
		Body.F[i][0] = x_temp;
		Body.F[i][1] = y_temp;
		Body.F[i][2] = F_sus[i][2];
	}
};
void Car_4wheel::BodyOutput()
{
	int i, j;
	for (i = 0; i < 4; i++)
	{
		z[i] = Body.z_sus[i];
		z_dot[i] = Body.zdot_sus[i];
		for (j = 0; j < 2; j++)
		{
			v_w[i][j] = Body.v_wheel[i][j];
		}
	}
};
void Car_4wheel::BodyUpdate()
{
	BodyreadF();
	//cout << "Fx_total:"<< Body.F[0][0]+Body.F[1][0]+Body.F[2][0]+Body.F[3][0]<<endl;
	//cout << "a:" <<Body.tmpmotion.a<<endl;
	Body.motionupdate(dt);
	
	Body.poseupdate(dt);
	
	Body.Outputz();
	
	Body.Calv_w();
	
	BodyOutput();
	
	if (Body.tmpmotion.a_side != 0)
		theta_ = Body.tmpmotion.v /( Body.tmpmotion.a_side*R);
	
}
void Car_4wheel::WheelInput()
{
	int i;
	double F;

	for (i = 0; i < 4; i++)
	{
		Wheel[i].tire.strAng = strAng[i];
		coordinate_change(v_w[i][0],v_w[i][1], strAng[i]);
		Wheel[i].tire.vXtire = x_temp;
		Wheel[i].tire.vYtire = y_temp;
		//cout << "Wheel[i].tire.vXtire  " << Wheel[i].tire.vXtire << endl;
		//cout << "Wheel[i].tire.vYtire  " << Wheel[i].tire.vYtire << endl;
//		if (abs(Wheel[i].tire.slipangle) < 0.05)
	//		Wheel[i].tire.slipangle = 0;

		Wheel[i].z = z[i];
		//cout << "Wheel[i].z "<< Wheel[i].z <<endl;
		Wheel[i].z_dot = z_dot[i];
		//cout << "Wheel[i].z_dot"<<Wheel[i].z_dot<<endl;
		Wheel[i].Z_ground = 0;
		Wheel[i].Zdot_ground = 0;

	}
	

/*
	if (std::abs(Wheel[0].tire.vXtire - Wheel[1].tire.vXtire) < 0.2)
	{
		F = (Wheel[0].tire.vXtire + Wheel[1].tire.vXtire) / 2;
		Wheel[0].tire.vXtire = F;
		Wheel[1].tire.vXtire = F;
	}
	if (abs(Wheel[2].tire.vXtire - Wheel[3].tire.vXtire) < 0.2)
	{
		F = (Wheel[2].tire.vXtire + Wheel[3].tire.vXtire) / 2;
		Wheel[2].tire.vXtire = F;
		Wheel[3].tire.vXtire = F;
	}
	if (abs(Wheel[0].tire.rolltire - Wheel[1].tire.rolltire)<0.5)
	{
		F = (Wheel[0].tire.rolltire + Wheel[1].tire.rolltire) / 2;
		Wheel[0].tire.rolltire = F;
		Wheel[1].tire.rolltire = F;
	}
	if (abs(Wheel[2].tire.rolltire - Wheel[3].tire.rolltire)<0.5)
	{
		F = (Wheel[2].tire.rolltire + Wheel[3].tire.rolltire) / 2;
		Wheel[2].tire.rolltire = F;
		Wheel[3].tire.rolltire = F;
	}*/
};
void Car_4wheel::Wheeloutput()
{
	int i;
	double F;
	for (i = 0; i < 4; i++)
	{
		coordinate_change(Wheel[i].tire.vXtire, Wheel[i].tire.vYtire, -strAng[i]);
		v_w[i][0] = x_temp;
		v_w[i][1] = y_temp;
		//cout << "T_shaft[i] - M_brake[i]  " << T_shaft[i] - M_brake[i] << endl;
		//cout <<"T_shaft[i]  " << T_shaft[i] << endl;
		//cout <<"M_brake[i]  " << M_brake[i] << endl;
		Wheel[i].suspstate();

		double temp = Body.Env1.f * Wheel[i].tire.Fztire*rw;
		Wheel[i].tirestate(T_shaft[i], M_brake[i], If, Body.Env1.f * Wheel[i].tire.Fztire*rw,dt,Car_4wheel::i_car*Diff.ig,1-i/2);
		
		//cout << "envf1  " <<Body.Env1.f << endl;
		//cout << "Fztire  " <<Wheel[i].tire.Fztire << endl;
		//cout << "rw  " << rw  << endl;


		n_w[i] = Wheel[i].tire.rolltire;
		slipratio[i] = Wheel[i].tire.slipratio;
		F_sus[i][2] = Wheel[i].suspension.Fzsusp;

		//cout << "slipratio:"<< Wheel[i].tire.slipratio<<endl;
		//cout << "Fxtire: "<< Wheel[i].tire.Fxtire<<endl;
	}


/*
	if (abs(Wheel[0].tire.Fxtire - Wheel[1].tire.Fxtire) < 50)
	{
		F = (Wheel[0].tire.Fxtire + Wheel[1].tire.Fxtire) / 2;
		Wheel[0].tire.Fxtire = F;
		Wheel[1].tire.Fxtire = F;
	}
	if (abs(Wheel[2].tire.Fxtire - Wheel[3].tire.Fxtire) < 50)
	{
		F = (Wheel[2].tire.Fxtire + Wheel[3].tire.Fxtire) / 2;
		Wheel[2].tire.Fxtire = F;
		Wheel[3].tire.Fxtire = F;
	}
	if (abs(Wheel[0].tire.Fztire - Wheel[1].tire.Fztire) < 30)
	{
		F = (Wheel[0].tire.Fztire + Wheel[1].tire.Fztire) / 2;
		Wheel[0].tire.Fztire = F;
		Wheel[1].tire.Fztire = F;
	}
	if (abs(Wheel[2].tire.Fztire - Wheel[3].tire.Fztire) < 30)
	{
		F = (Wheel[2].tire.Fztire + Wheel[3].tire.Fztire) / 2;
		Wheel[2].tire.Fztire = F;
		Wheel[3].tire.Fztire = F;
	}*/
	for (i = 0; i < 4; i++)

	{
		Wheel[i].Foutput(T_shaft[i], M_brake[i], 1 - i / 2);
		F_sus[i][0] = Car_4wheel::Wheel[i].tire.Fxtire;
		F_sus[i][1] = Car_4wheel::Wheel[i].tire.Fytire;
		F_sus[i][2] = Wheel[i].suspension.Fzsusp;
	}
	n_ = (n_w[0] + n_w[1])/2 * 60/ (2.0 * PI);
};
void Car_4wheel::stroutput()
{
	if(abs(strwheel)<0.02)
	{
		strwheel=0;
	}
	Str.Ackermansteering(strwheel);
	strAng[0] = Str.Ang_fl;
	strAng[1] = Str.Ang_fr;
	strAng[2] = 0;
	strAng[3] = 0;
};
void Car_4wheel::CalR()
{
	if (Str.Ang != 0)
	{
		double R1 = (Body.lf + Body.lr) / tan(Str.Ang);
		R = abs(Str.Ang) / (Str.Ang)*pow((pow(R1, 2) + pow(Body.lr, 2)), 0.5);
	}
	else
		R = 0.0001;
}
void Car_4wheel::Engoutput()
{
	double T = 0;
	Trans.n = n;
	//cout << "throttlepedal"<<throttlepedal<<endl;
	if (throttlepedal>0.1)
		Trans.clutch=true;
	if (Trans.clutch==true)
	{
		Trans.autotrans();
		i_car = Trans.i_;
		//cout << "n_  " << n_ << endl;
		//cout << "icar   " << i_car << endl;
		if (std::abs(n_)>60)
			n = n_ * (Diff.ig*i_car);
		else
		    n=800;
		Eng.EngineTor(throttlepedal, n);
		T = Eng.Torq*i_car;
		Trans.n = n;
		Diff.diffstate(n_w[1]*60/(2*3.1415926), n_w[0]*60 / (2 * 3.1415926), T);
		T_shaft[0] = Diff.diffst.Tl;
		T_shaft[1] = Diff.diffst.Tr;
		T_shaft[2] = 0;
		T_shaft[3] = 0;
	}
	else
	{
		if (n > 800)
		n -= dt * 1000;
		T_shaft[0] = 0;
		T_shaft[1] = 0;
		T_shaft[2] = 0;
		T_shaft[3] = 0;
	}

};
void Car_4wheel::Brakeoutput()
{
	//cout <<" "<< theta_ << endl;
	int i = 0;
	Brake.rw = Body.rw;
	for (i = 0; i < 4; i++)
	{
		Brake.slip[i] = slipratio[i];
		Brake.n[i] = n_w[i];
		Brake.v_w[i] = Wheel[i].tire.vXtire;
	}
	Brake.brakeInput(brakepedal);

	if (brakepedal > 0.05)
	{
        if (abs(Body.tmpmotion.v) > 0.05)
        {
            //Brake.ABScontrol();
            //Brake.ESPcontrol(theta_);
            Brake.Output();
            for (i = 0; i < 4; i++)
            {
                M_brake[i] = Brake.M[i];
            }
        }
        else
        {
            Brake.Output();
            for (i=0;i<4;i++){
                M_brake[i]=Brake.M[i]*abs(Body.tmpmotion.v)/0.05;
            }

            Trans.transinit();
            n=800;
        }
		T_shaft[0] = 0.0;
		T_shaft[1] = 0.0;
		Trans.clutch = false;
	}
	else
	{
        for (i = 0; i < 4; i++)
        {
            M_brake[i] = 0 ;
        }
	}
};

void Car_4wheel::Carupdate(double time_gap)
{
       // dt = time_gap;
        std::cout << " dt : " << dt << std::endl;
	BodyUpdate();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 1" << std::endl;
	}

	Engoutput();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 2" << std::endl;
	}
	Brakeoutput();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 3" << std::endl;
	}
	stroutput();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 4" << std::endl;
	}
	CalR();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 5" << std::endl;
	}
	WheelInput();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 6" << std::endl;
	}
	Wheeloutput();
	if (isnan(Body.tmpmotion.x)) {
		//std::cout << "xxx: 7" << std::endl;
	}
	output[0] = Body.tmpmotion.x;
	output[1] = Body.tmpmotion.y;
	output[2] = Body.tmpmotion.Yaw;
	output[5] = Body.tmpmotion.dYawdt;
	output[3] = Body.tmpmotion.v*cos(Body.tmpmotion.Yaw + Body.tmpmotion.phi);
	output[4] = Body.tmpmotion.v*sin(Body.tmpmotion.Yaw + Body.tmpmotion.phi);
	//cout << "SNS"<<endl;
	/*
    cout << "Trans.clutch "<<"  "<<Trans.clutch << endl;
    //cout << "Brake Motion "<<"  "<<M_brake[0]<<"  "<<M_brake[1]<<"    "<<M_brake[2]<<"    "<<M_brake[3]<<endl;
    cout << "i_car "<<" "<< i_car<<endl;
    cout << "Engine Speed "<< " "<<n<<endl;
    cout << "ROLLTIRE"<<"   "<<n_<<endl;
    cout << "Velocity"<<"   "<<Body.tmpmotion.v<<endl;
    cout << "rolltire_fl"<<"	"<<Wheel[0].tire.rolltire<<endl;
	cout << "rolltire_fr"<<"	"<<Wheel[1].tire.rolltire<<endl;
	cout << "strwheel" <<"  "<<strwheel<<endl;
	cout << "StrRatio"<<"   "<<Str.Ratio<<endl;
	cout << "wheel steering angle _fl"<<"   "<<strAng[0]<<endl;
    cout << "wheel steering angle _fr"<<"   "<<strAng[1]<<endl;
    cout << "vYtire"<<" "<<Wheel[0].tire.vYtire<<endl;
    cout << "vYtire"<<" "<<Wheel[1].tire.vYtire<<endl;
    cout << "vYtire"<<" "<<Wheel[2].tire.vYtire<<endl;
    cout << "vYtire"<<" "<<Wheel[3].tire.vYtire<<endl;
	cout << "Side Force"<<" "<<Wheel[0].tire.Fytire<<endl;
    cout << "Side Force"<<" "<<Wheel[1].tire.Fytire<<endl;
    cout << "Side Force"<<" "<<Wheel[2].tire.Fytire<<endl;
    cout << "Side Force"<<" "<<Wheel[3].tire.Fytire<<endl;
    cout << "Side acceleration"<< " "<<Body.tmpmotion.a_side<<endl;
    cout << "dYaw/dt "<< " "<<Body.tmpmotion.dYawdt<<endl;
    cout << "Yaw"<<"    "<<Body.tmpmotion.Yaw<<endl;
    cout << "   "<<endl;*/
};

void Car_4wheel::getinput(double throttle_,double brake_,double strwheel_)
{
	throttlepedal = throttle_;
	brakepedal = brake_;
	strwheel = strwheel_;
};
/*
void Car_4wheel::updatetick(int argc,double argv[])
{
	static int count = 0;
	if (count == 0)
		Carinit(argv[0], argv[1], argv[2], argv[3], argv[4], argv[5]);
	else
	{
		getinput(argv[0], argv[1], argv[2]);
		Carupdate();
	}
	count++;
}
*/
