
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include <cstdio>
#include <vector>
#include <utility>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_matching/LaneletMatching.h>


// #include "pure_pursuit.hpp"
using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace lanelet;
using namespace lanelet::matching;

namespace PurePursuit {

	// vector<goalPoint> Goalpoint;
	// double			 RatioToReal = 1.0;  //the pixel axis 's ratio to the real axis
	// const double LookAheadDistance = 8;
	// const double Speed = 4.0;
	// const double distanceWithinGoal = 1;
	//const double LengthOfBicycleModel = 3.8 ;
	// double			 WidthOfCar = 1.92 ;
	// double           LengthOfCar = 4.72 ;
	// double           radius_car; //use three circles to approximate our car


extern int solve_cubic(float a, float b, float c, float d, float* xout);

double signum(double n);
double minvalue(double a, double b) ;

double maxvalue(double a, double b) ;

double find_min_amongThreeA(double a, double b, double c) ;

// //pure pursuit controller-based smoother
// void FindCurrentLocation() {

//     RealPath.push_back(CurrentPoint);  //save the first point
// 	while (Goalpoint[Goalpoint.size() - 1].flag != 1) {
// 		for (int i = 0; i < Goalpoint.size(); i++) {
// 			double distanceTOGoal = sqrt(pow(Goalpoint[i].x() - CurrentPoint.x(), 2) + pow(Goalpoint[i].y() - CurrentPoint.y(), 2));
// 			if (distanceTOGoal < distanceWithinGoal) {
// 				Goalpoint[i].flag = 1;
// 			}
// 		}

// 		pair_d lookahead;
// 		// get lookahead point
// 		getLookaheadPoint(lookahead);
// 		// update bicycle model state
// 		CalcBicycleModel(lookahead);

// 		/*
// 		// Transform the goalpoint to vehicle coordinates
// 		double offsetX = lookahead.first - CurrentPoint.x();
// 		double offsetY = lookahead.second - CurrentPoint.y();

// 		double distanceToPoint = sqrt(offsetX*offsetX + offsetY * offsetY);
// 		double normalizedX = offsetX / distanceToPoint;
// 		double normalizedY = offsetY / distanceToPoint;

// 		CurrentPoint.x() += normalizedX * Speed;
// 		CurrentPoint.y() += normalizedY * Speed;
// 		RealPath.push_back(CurrentPoint);*/
// 	}

// }

//compute the lookahead point(goal point) by whether line and cirle intersection
//Circle-line intersection. http://mathworld.wolfram.com/Circle-LineIntersection.html
BasicPoint2d getLookaheadPoint(BasicPoint2d& currentPoint, vector<BasicPoint2d>& goalPointpoints, double currentTheta) ;

//bicycle model of a car
BasicPoint2d CalcBicycleModel(BasicPoint2d& lookahead, BasicPoint2d& currentPoint, double currentTheta) ;

LineString2d getSmoothPath(ConstLanelet originalPath, double currentTheta) ;

} // End of namespace


