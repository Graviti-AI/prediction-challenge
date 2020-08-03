#pragma once
#include <vector>
#include "Global_parameter.h"

using namespace std;

struct sLattice_point {
	sLattice_point(void){}
	sLattice_point(double xx, double yy, double ttheta, double kkappa, double ss) {
		x = xx;
		y = yy;
		theta = ttheta;
		kappa = kkappa;
		s = ss;
	}
	double x;
	double y;
	double kappa;
	double s;
	double theta;
};

struct sLattice_dot {
	sLattice_dot(void){}
	sLattice_dot(double xx, double yy) {
		x = xx;
		y = yy;
	}
	double x;
	double y;
};

// Cubic curve that expressed by x & y.
struct cubicCurveXY{
	double p0;
	double p1;
	double p2;
	double p3;
	sLattice_dot startPoint;
	sLattice_dot endPoint;
};	

// Cubic curve that expressed by \kappa & \theta.
struct cubicCurveKS {
	double p0;
	double p1;
	double p2;
	double p3;
	sLattice_dot startPoint;
	sLattice_dot endPoint;
	double length;
	double theta;
	double startS;
	double endS;
};	

// Cubic curve that expressed by parameter: x(s) & y(s)
struct cubicCurvePara {
	double a0;
	double a1;
	double a2;
	double a3;
	double b0;
	double b1;
	double b2;
	double b3;
	double startS;
	double endS;
	double length;
	sLattice_dot startPoint;
	sLattice_dot endPoint;
};	

class SpatialLattice {
public:
	// Input
	vector<sLattice_point> goals;
	double width = LATTICE_WIDTH;

	// Equations of the curve. [lattice][curve].
	vector<vector<cubicCurvePara>> spatialLatticePara;
	int numBeginningCurve;

	// Functions
	void init();				// Initialize the class, must be run every time after define an object.
	bool mainSpatialLattice();	// To get the equations. Return to spatialLatticePara.
	double kappaInfo(int num, double s);	// To calculate kappa(s) on the "num" lattice.
	// @ brief: return the x coordinate of the point with curve length s, it's actually the function X(s)
	// @ attention: the s means s in the local lattice instead of the global lattice
	// @ input: num: the index of the route
	//          s: x(s)
	// @ output: x coordinate
	double XInfo(int num, double s);

	// @ brief: return the y coordinate of the point with curve length s, it's actually the function Y(s)
	// @ attention: the s means s in the local lattice instead of the global lattice
	// @ input: num: the index of the route
	//          s: y(s)
	// @ output: y coordinate
	double YInfo(int num, double s);

	// @ brief: return the theta angle of the point with curve length s, it's actually the function theta(s)
	// @ attention: the s means s in the local lattice instead of the global lattice
	// @ input: num: the index of the route
	//          s: theta(s)
	// @ output: theta angle
	double ThetaInfo(int num, double s);

private:
	const int num1stLayer = NUM1STLAYER;	// must be an odd number.
	const int num2ndLayer = NUM2NDLAYER;	// must be an odd number.
	vector<sLattice_dot> nodes1st;	// Position of the nodes on the 1st layer.
	vector<sLattice_dot> nodes2nd;	// Position of the nodes on the 2nd layer.

	// Equations of the curve. [lattice][curve].
	vector<vector<cubicCurveXY>> spatialLatticeXY;
	vector<vector<cubicCurveKS>> spatialLatticeKS;

	sLattice_dot coordinateChange(sLattice_dot points, bool flag, double theta);	// Rotate the coordinate system. flag = 1: rotating; flag = 0: rerotating.

	// Calculate the curvature, arc-length and angle information for spatial nodes. 
	//*Input the position, the curve info, and the start point.
	double kappaCal(sLattice_dot a, cubicCurveXY curve, sLattice_dot start);	
	double sCal(sLattice_dot a, cubicCurveXY curve, sLattice_dot start);
	double thetaCal(sLattice_dot a, cubicCurveXY curve, sLattice_dot start);

	// Interpolation functions, using the 1st derivative boundary.
	//*Input the position of the dots and left and right derivative.
	vector<cubicCurveXY> interp_xy(vector<sLattice_dot> &dots, double boundL, double boundR);
	vector<cubicCurveKS> interp_ks(vector<sLattice_point> &dots, double boundL, double boundR);
	vector<cubicCurvePara> interp_para(vector<sLattice_point> &dots, double thetaL, double thetaR);

	// Functions to get the explicit expression of the curve.
	//*Input 4 goal points, including the x, y, s, curvature and angle information, and the total width of the 1st layer.
	vector<vector<cubicCurveXY>> generateSpatialLattice(vector<sLattice_point> &goals, double width);
	vector<vector<cubicCurveKS>> generateEquationKS(vector<sLattice_point> &goals, double width);
	vector<vector<cubicCurvePara>> generateEquationPara(vector<sLattice_point> &goals, double width);
};