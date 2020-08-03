#pragma once

#include <vector>
#include <string>

using namespace std;
typedef pair<double, double> pair_d;

struct zhaotinggoalPoint {
	double _x;
	double _y;
	double _theta;
	double flag;
	zhaotinggoalPoint(void) {};		//why?
	zhaotinggoalPoint(int x, int y, int theta,int f) {
		_x = x;
		_y = y;
		_theta = theta;
		flag = f;
	}
};

struct zhaotingpathPoint {
	double _x;
	double _y;
	double _theta;
	double _v;
	double _s;   // arc length
	double _curvature;
	double _dmin; // min distance to obstacles
	zhaotingpathPoint(void) {};
	zhaotingpathPoint(int x, int y, int theta) {
		_x = x;
		_y = y;
		_theta = theta;
	}
};


class zhaotingPurePursuit
{
public:
	vector<zhaotinggoalPoint> Goalpoint;		//goalPoint is struct defined above
	double			 RatioToReal = 1.0;  //the pixel axis 's ratio to the real axis
	double           	 LookAheadDistance = 4;
	double           	 Speed = 10;
	double			 dt = 0.01;
	double 			 distanceWithinGoal = 1;
	double           LengthOfBicycleModel = 3.8 ;
	double			 WidthOfCar = 1.92 ;
	double           LengthOfCar = 4.72 ;
	double           radius_car; //use three circles to approximate our car
		
    double           Speed_now;

    double           Total_length ;


	zhaotingpathPoint		  CurrentPoint;		//used by bicycle model smoother
	zhaotingpathPoint         HistoryPoint;
	pair_d            History_acc;
	zhaotingpathPoint		  CurrentCar;     //used to record the current car state

	vector<zhaotingpathPoint>   CurrentCarSet;
	vector<zhaotingpathPoint>	RealPath;
	vector<zhaotingpathPoint>   DownSampledPath;


	//vector<vector<costOFone>>   TrajectoryBucketSearch5;
	void FindCurrentLocation();   //determine the current location of the vehicle 
	void getLookaheadPoint(pair_d &lookahead);
	void CalcBicycleModel(pair_d lookahead);
	
   
    double signum(double n);
    double minvalue(double a, double b);
    double maxvalue(double a, double b);
};
