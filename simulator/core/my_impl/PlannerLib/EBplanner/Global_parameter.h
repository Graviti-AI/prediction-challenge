#ifndef LANELET2_GLOBAL_PARAMETER_H
#define LANELET2_GLOBAL_PARAMETER_H

#endif //LANELET2_GLOBAL_PARAMETER_H

//----------------------------Attention---------------------------------------
//In real car experiment, need to check these files
//1.EBplannerA.cpp   set the rate of EB planner to 10 hz
//2.vel_extract.py   extract the 5 th point
//3.PurePursuit.h    the control_T should be set to 0.1s
//4.In real car experiment, it is possibile that the feedback speed value is not percise, in this case, the planner will failed and 
//  do not forget to change the initial acceleration to be zero.
//5.Due to the safety reason, the initial acceleration is limited to bounded area, this bounded area can be changed to a more realiable value

//the path need to be changed:
//1.MapReader.cpp
//2.send_obs_info/main.cpp
//3.lanelet_ref_read/main.cpp

// Speed Planner
#define SPEED_PLANNER_FREQUENCY                 10.0

// Interactive Planner
#define SHORT_TERM_HORIZON                      0.5
#define TOTAL_HORIZON                           3.0  
#define SPEED_SAMPLE_STEP                       0.5  // the step length between each sample point in short-term speed sample
#define MAX_SPEED_SAMPLED                       10.0  // the maximum sampled speed for both short-term and long-term horizon
#define MIN_SPEED_SAMPLED                       0.0  // the minimum
#define TIME_RESOLUTION                         0.1  // the time interval of computing the cost, eg, compute the location of our car after every 0.1 s to get a car trajectory
#define NUM_EGO_NON_STOP_SPEED_SAMPLED          3    // the number of long-term non stop speed curve we want to sample
#define NUM_EGO_STOP_SPEED_SAMPLED              2    // the number of long-term stop speed curve we want to sample
#define NUM_OBSTACLE_NON_STOP_SPEED_SAMPLED     7    // it's the same but for the obstacle car
#define NUM_OBSTACLE_STOP_SPEED_SAMPLED         2    // it's the same but for the obstacle car


#define OBSTACLE_MAX_SPEED_SAMPLED              10.0
#define OBSTACLE_MIN_SPEED_SAMPELD              0.0
#define OBSTACLE_TIME_STEP                      0.1 

#define COLLISION_DISTANCE_THRESHOLD            0.00001
#define COLLISION_PROBABILITY_THRESHOLD         0.5

// EdgeAugumented_Planner
#define	N_COLUMN 20                         // number of column of spatial graph
#define	N_ROW 11                            // number of row of spatial graph
#define	EDGE_NUMBER 5                       // the freedom of a spatial node
#define	ROAD_WIDTH 4.0
#define MAX_RADIUS_ 4.0                     // max radius of each spatial node-->max safe distance of a node
#define MIN_DISTANCE_TO_OBSTACLES 1.0       // the min distance to obstacels we permit,
                                            //*nodes whose distance is smaller than this value should be deleted during update
#define ATTRACT_FORCE_GAIN_ 1.0
#define REPULISIVE_FORCE_GAIN_ 2.0
#define REPULISIVE_FORCE_SPEED_GAIN 0.3
#define CONTRACTION_FORCE_GAIN_ 5.0
#define STEP_SPATIALNODE 0.2

// PurePursuit
#define RATIO_TO_REAL 10.0                  //the pixel axis 's ratio to the real axis
#define LOOK_AHEAD_DISTANCE 0.3             // origin value: 5
#define SPEED 1.0                           //the speed of pure pursuit smoother
#define DISTANCE_WITHIN_GOAL 0.5            //pure pursuit controller--> used to decide whether the goal point has been reached
#define LENGTH_OF_BICYCLE_MODEL 3.8         //pure pursuit controller--> the length of the bicycle model
#define WIDTH_OF_CAR 1.92                    //width of my car
#define LENGTH_OF_CAR 4.72                  //length of my car
#define MAX_DISTANCE_CONSIDERING_OBSTACLES 8.0
#define OBSTACLE_SPEED -2.0                 //obstacle speed predicted(constant) -- not used
#define MAX_LATERAL_ACCELERATION 1.0
#define MAX_LONGITUDINAL_DECELERATION 3.0
#define MAX_LONGITUDINAL_ACCELERATION 1.5
#define MAX_SPEED 7.5//4.76 // origin 7.0 4.76
#define MAX_JERK 2.0
#define MAX_OBSTACLE_PROXIMITY 0.8        //ratioToreal
#define MAX_Following_Distance 3.0
#define MAX_SAFE_DISTANCE 10.0
#define MAX_SAFE_SPEED  4.0                 //used to decide whether this speed profile is safe when the obstacle is near our car
#define MAX_SPATIAL_DIFF_FROM_REF 1.0
#define MAX_TEMPRAL_DIFF_FROM_REF 2.0
#define TIME_SAMPLING_HORIZON 5.0

//cascaded ranking--> bucket size
//~~~~~~~~~~~~the priority should be modified at purepursuit.cpp         //priority
#define           StaticObjs_BucketValue1     30.0                       //0
#define           StaticObjs_BucketValue2     50.0
#define           MovingObjs_BucketValue1     200.0                       //1
#define           MovingObjs_BucketValue2     500.0
#define           LongAcc_BucketValue1        100.0                      //5
#define           LongAcc_BucketValue2        300.0
#define           LatAcc_BucketValue1         200.0                     //3
#define           LatAcc_BucketValue2        400.0
#define           SpatialDiff_BucketValue1    800.0                      //4
#define           SpatialDiff_BucketValue2    1000.0
#define           TemporalDiff_BucketValue1   100.0                      //2
#define           TemporalDiff_BucketValue2   200.0


// Global   -->opencv image size
#define G_ROWS_NUM 1200
#define G_CLOS_NUM 1200


// main
#define REFERENCE_LENGTH 30.0    //the length of the reference used to create discrete elastic band

// Draw
#define TRANSFORM_TRANS_X -558550*5
#define TRANSFORM_TRANS_Y -4196540*5
#define TRANSFORM_SCALE_X 5.0
#define TRANSFORM_SCALE_Y 5.0

// SpatialLattice
#define NUM1STLAYER 1	                    // must be an odd number.
#define NUM2NDLAYER 1	                    // must be an odd number.
#define LATTICE_WIDTH 0.6

// ElasticBand --not used
#define NUM_OPTIM_ITERATIONS_ 8             //the number of iterations when doing optim
#define CENTERLINE_Y 200.0                  //the y location of centerline-->used to compute attract forces
#define CAR_LENGTH 2.0 * 18.0
#define CAR_WIDTH 2.0 * 6.48
#define TINY_BUUBLE_EXPANSION 2.0           //lower bound for bubble expansion. below this bound bubble is considered as "in collision"
#define ATTRACT_FORCE_GAIN 0.005
#define INTERNAL_FORCE_GAIN 1.0
#define EXTERNAL_FORCE_GAIN 1.0
#define MAX_BUBBLE_RADIUS 20.0
#define MIN_BUBBLE_OVERLAP 0.9
#define STEPSIZE_ 2.0                       //the step size of [the min distance form bubble to obstacle]