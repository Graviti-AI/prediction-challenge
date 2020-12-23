#include "speed_planner.h"
#include<deque>
#include "threadpool.h"
#include "MDN.h"
#include "GMM.h"
#include "KMeans.h"
#include "Eigen/Core"
#include "Eigen/Dense"

using namespace Eigen;

#undef CLAMP
#define CLAMP(x, low, high) (((x) > (high))? (high): (((x) < (low))?(low):(x)))

const bool threadpool_flag = false; // TODO:
const bool courteous_flag = false;
const bool conservative_flag = false;
const bool pure_prediction_flag = false;

const double courteous_term = 2;
static bool no_collision_flag; //= false;
static bool no_short_term_flag; //= false;
static int fail_time = 0;

// for experiment use
static string Speed_planner_index_number;
static string logtxt_path;
static string cost_detail_txt_path;
static string long_term_pro_cost_txt_path;

class SpeedParam
{
 public:
    double a3;
    double a2;
    double a1;
    double a0;
    double stop_time = TOTAL_HORIZON;
    double GetSpeed(double t)
    {
        if(t > stop_time)
        {
            return 0.0;
        }
        else
        {
            return a3 * pow(t, 3) + a2 * pow(t, 2) + a1 * t + a0;
        }

    }
    double GetAcceleration(double t)
    {
        if(t > stop_time)
        {
            return 0.0;
        }
        else
        {
            return 3.0 * a3 * pow(t,2) + 2.0 * a2 * t + a1;
        }
    }
    double GetDistanceCovered(double t)
    {
        if(t > stop_time)
        {
            return GetDistanceCoveredNoneStop(stop_time);
        }
        else
        {
            return GetDistanceCoveredNoneStop(t);
        }
    }
    double GetJerk(double t)
    {
        if(t > stop_time)
        {
            return 0.0;
        }
        else
        {
            return 6.0 * a3 + 2.0 * a2;
        }
    }
    double GetAverageSpeed(double t0, double t1)
    {
        double s0 = GetDistanceCovered(t0);
        double s1 = GetDistanceCovered(t1);
        return (s1 - s0) / (t1 - t0);
    }
    void GetStopInfo()
    {
        for(double t = 0.0; t < TOTAL_HORIZON; t += TIME_RESOLUTION)
        {
            double v = GetSpeed(t);
            if ( v < 0)
            {
                stop_time = max(0.0, t - TIME_RESOLUTION);;
                break;
            }
        }
    }
 private:
    double GetDistanceCoveredNoneStop(double t)
    {
        return 1.0 / 4.0 * a3 * pow(t,4) + 1.0 / 3.0 * a2 * pow(t,3) + 1.0 / 2.0 * a1 * pow(t,2) + a0 * t;
    }

};


class ShortTermSpeedCurve
{
 public:
    SpeedParam speed_param;
    double cost = 0;
    bool operator <(const ShortTermSpeedCurve& speed_curve_2)
    {
        if(cost < speed_curve_2.cost)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

class CompleteSpeedCurve  // one short-term speed plus a long-term speed to become a complete speed curve
{
 public:
    SpeedParam short_term_speed_param;
    SpeedParam long_term_speed_param;
    Cost cost;
    double overall_cost = 0; // the cost after basic ranking w1*cost1 + w2 * cost2....
    double stop_time = TOTAL_HORIZON;  // the time when the velocity becomes 0, which means the car stop
    // the stop time will be updated after calling computecostexceptmovingobstacle
    double GetSpeed(double t)
    {
        if(t > stop_time)
        {
            return 0.0;
        }
        else
        {
            if(t < SHORT_TERM_HORIZON)
            {
                return short_term_speed_param.GetSpeed(t);
            }
            else if (t <= TOTAL_HORIZON)
            {
                return long_term_speed_param.GetSpeed(t - SHORT_TERM_HORIZON);
            }
            else
            {
                std::cout << "Error, you input a time that's not in the right range to compute the velocity\n";
            }
        }
    }
    double GetAcceleration(double t)
    {
        if(t > stop_time)
        {
            return 0.0;
        }
        else
        {
            if(t < SHORT_TERM_HORIZON)
            {
                return short_term_speed_param.GetAcceleration(t);
            }
            else if (t <= TOTAL_HORIZON)
            {
                return long_term_speed_param.GetAcceleration(t - SHORT_TERM_HORIZON);
            }
            else
            {
                std::cout << "Error, you input a time that's not in the right range to compute the acceleration\n";
            }
        }
    }
    double GetJerk(double t)
    {
        if(t > stop_time)
        {
            return 0.0;
        }
        else
        {
            if(t < SHORT_TERM_HORIZON)
            {
                return short_term_speed_param.GetJerk(t);
            }
            else if (t <= TOTAL_HORIZON)
            {
                return long_term_speed_param.GetJerk(t - SHORT_TERM_HORIZON);
            }
            else
            {
                std::cout << "Error, you input a time that's not in the right range to compute the jerk\n";
            }
        }
    }
    double GetDistanceCovered(double t)
    {
        if(t < stop_time)
        {
            return GetDistanceCoveredNoneStop(t);
        }
        else if (t <= TOTAL_HORIZON)
        {
            return GetDistanceCoveredNoneStop(stop_time);
        }
        else
        {
            std::cout << "Error, you input a time that's not in the right range to compute the distance covered\n";
        }
    }
    double GetAverageSpeed(double t0, double t1)
    {
        double s0 = GetDistanceCovered(t0);
        double s1 = GetDistanceCovered(t1);
        return (s1 - s0) / (t1 - t0);
    }
    // sum of weight * cost
    double CalculateOverallCost()
    {
        double moving_obstacles_weight = 0;
        // double moving_obstacles_weight = 0;
        double lateral_acceleration_weight = 0.4; //0.0;
        double longitudinal_acceleration_weight = 0.08; // 0.0;
        double jerk_weight = 0.03; //0.0;
        double temporal_difference_weight = 0.1; // 0.2;
        // cout << "obstacle weight: "<<cost.moving_obstacles_cost << endl;
        // cout << "after: " << moving_obstacles_weight * cost.moving_obstacles_cost <<endl;
        overall_cost = moving_obstacles_weight * cost.moving_obstacles_cost +
                       lateral_acceleration_weight * cost.lateral_acceleration_cost +
                       longitudinal_acceleration_weight * cost.longitudinal_acceleration_cost +
                       jerk_weight * cost.jerk_cost +
                       temporal_difference_weight * cost.temporal_diff_from_ref_cost;
        
        // ofstream cost_detail("/home/interaction/cost_detail.txt", ios::app);
        // cost_detail << "lateral_acceleration_cost: " <<  lateral_acceleration_weight * cost.lateral_acceleration_cost << endl;
        // cost_detail << "longitudinal_acceleration_cost: " <<  longitudinal_acceleration_weight * cost.longitudinal_acceleration_cost << endl;
        // cost_detail << "jerk_cost: " <<  jerk_weight * cost.jerk_cost << endl;
        // cost_detail << "temporal_diff_from_ref_cost: " <<  temporal_difference_weight * cost.temporal_diff_from_ref_cost << endl<<endl;
        // cost_detail.close();

        // cout << "lateral_acceleration_cost: " <<  lateral_acceleration_weight * cost.lateral_acceleration_cost << endl;
        // cout << "longitudinal_acceleration_cost: " <<  longitudinal_acceleration_weight * cost.longitudinal_acceleration_cost << endl;
        // cout << "jerk_cost: " <<  jerk_weight * cost.jerk_cost << endl;
        // cout << "temporal_diff_from_ref_cost: " <<  temporal_difference_weight * cost.temporal_diff_from_ref_cost << endl<<endl;
        return overall_cost;
    }
    bool operator <(const CompleteSpeedCurve& speed_curve_2)
    {
        if(overall_cost < speed_curve_2.overall_cost)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void GetStopInfo()
    {
        for(double t = 0.0; t < TOTAL_HORIZON; t += TIME_RESOLUTION)
        {
            double v = GetSpeed(t);
            if ( v < 0)
            {
                stop_time = max(0.0, t - TIME_RESOLUTION);
                break;
            }
        }
    }
 private:
    double GetDistanceCoveredNoneStop(double t)
    {
        if(t < SHORT_TERM_HORIZON)
        {
            return short_term_speed_param.GetDistanceCovered(t);
        }
        else if (t <= TOTAL_HORIZON)
        {
            return short_term_speed_param.GetDistanceCovered(SHORT_TERM_HORIZON) + long_term_speed_param.GetDistanceCovered(t - SHORT_TERM_HORIZON);
        }
        else
        {
            std::cout << "Error, you input a time that's not in the right range to compute the distance covered\n";
        }
    }

};



class NonconservativePlanner: public SpeedPlanner
{
 public:
    double 			  delta_t = TIME_RESOLUTION;
    double            last_acceleration = 0.0;
    double 			  obstacle_current_speed = 0.0;
    double            obstacle_last_speed = 0.0;
    double            last_speed = 0.0;
    double            ego_current_x = 0;
    double            ego_current_y = 0;
    double            obs_current_x = 0;
    double            obs_current_y = 0;
    vector<CompleteSpeedCurve> ego_speed_params;
    vector<vector<double> > ego_speed_plot_points;
    vector<vector<double> > obstacle_speed_plot_points;
    // the route lattice
    SpatialLattice lattice;

    vector<cubicCurvePara> SpatialSamplingResults;
    SpeedParam  speed_result;

    vector<car_obstacle_> obstacle_car;
    // @ brief: this function uses non-conservative planning method to plan for only the speed profile in the next period
    // @ param: end_point_index: the array index of the end_point in the realpath array, which is the path generated by the pure pursuit
    // @ output: whether this algorithm has succeed or not
    bool PlanSpeed(int end_point_index);
    NonconservativePlanner()
    {
        num_sampled = int(double(MAX_SPEED_SAMPLED - MIN_SPEED_SAMPLED) / double(SPEED_SAMPLE_STEP)) + 1;
        cout << "the number of speed points sampled for the ego car is: " << num_sampled << endl;
        // InitGMM();
    }
    void GetSamplePoint();
 private:

    GMM  gmm;

    vector<double> ego_end_speed;
    vector<double> ego_stop_time;
    vector<double> obstacle_end_speed;
    vector<double> obstacle_stop_time;
    MDN mdn;

    vector<SpeedParam> obstacle_speed_curve_set;

    SpatialLattice obstacle_lattice;
    int num_sampled;                 // the total num of sample points of each term
    int min_obstacle_time_index = 0; // min size of the second layer of CarObstacleSet
    double predict_horizon = 0.0;
    // vector<double> raw_training_data; //  the raw training data for gmm
    // void InitGMM();

    // @ brief: this function plan the speed when there is no obstacle car;
    vector<ShortTermSpeedCurve> NaivePlanner();

    // @ when there is obstacle in the roundabout, the ego car stops
    vector<ShortTermSpeedCurve> ConservativePlanner();

    void GetMinObstacleTime();
    // @ brief: this function computes the cost of the car that has nothing to do with the obstacle car. for example, acceleration, jerk
    // the result will be stored in the speed_curve
    // @ input: one complete speed curve
    bool ComputeCostExceptMovingObstacleCost (CompleteSpeedCurve &speed_curve);

    // @ brief: this function computes the cost of obstacle car that has nothing to do with the ego car. for example, acceleration, jerk
    // the result will be returned
    // @ input: one complete speed curve
    double ComputeCostExceptMovingEgoCost (SpeedParam &speed_curve);

    // @ brief; this function choose 4 points and inteperlate the route to get an analytical representation of the route, which can be used for
    // a calculation in the speed planning part
    // @ param: end_point_index: the array index of the end_point in the realpath array, which is the path generated by the pure pursuit
    // @ output: whether this algorithm has succeed or not
    bool InterpolateTheRouteForSpeedPlanning(int end_point_index);

    // @ brief: it does the same thing as the previous function but this is for the obstacle car
    bool InterpolateTheRouteForObstacle();

    // @ brief: this function compute the cubic cure of a speed profile, the constraints are: initial speed, end speed, the initial acceleration
    // @ should equal to the history acceleration, and the jerk at the end should be 0
    // @ param: v0: initial speed
    //		    v1: end speed
    //          a: the acceleration that should be at this instant
    //          history_a: the acceleration in the last computation loop
    // output : the parameters of the speed curve v = a3*t^3 + a2*t^2 + a1*t + a0
    SpeedParam GenerateShortTermSpeedCurve(double v0, double v1, double a, double history_a);

    // @ brief: this function compute the cubic cure of a speed profile, the constraints are: initial speed, end speed, the initial acceleration
    // should equal to the history acceleration, and the accelerations at the end should be 0
    // @ param: v0: initial speed
    //		    v1: end speed
    //          a: the acceleration that should be at the beginning
    // @ output : the parameters of the speed curve v = a3*t^3 + a2*t^2 + a1*t + a0
    SpeedParam GenerateLongTermSpeedCurve(double v0, double v1, double a, double T);

    // @ brief: this function compute the cubic cure of a speed profile, the constraints are: initial speed, stop_time, the initial acceleration
    // should equal to the history acceleration, and the jerk at the end should be 0
    // @ param: v0: initial speed
    //		    stop_time: the time that the car should stop
    //          a: the acceleration that should be at the beginning
    // @ output : the parameters of the speed curve v = a3*t^3 + a2*t^2 + a1*t + a0
    SpeedParam GenerateLongTermStopSpeedCurve(double v0, double stop_time, double a);

    // @ brief:  given a short term speed curve, we use our interactive speed planner to compute the cost of this short-term speed curve
    // @ param: the short term speed curve
    bool ComputeShortTermSpeedCurveCost(ShortTermSpeedCurve& short_term_speed_curve);

    // @ brief:  given a groundtruth ego, we use prediction algorithms to calculate the probabilities of 9 obstacle trajectories
    void ComputeProbability();

    // @ brief: input two speed curve and get the likelihood
    // @ param: ego_car_speed_curve: the complete speed curve of an ego car
    //          obstacle_car_speed_curve: the speed curve of an obstacle car. Only one cubic curve
    //          obstacle_car_index: the index of the obstacle car
    // @ output: the likelihood p(X,Y)
    void GetLikelihoodMDN(CompleteSpeedCurve ego_car_speed_curve, SpeedParam obstacle_car_speed_curve, double & likelihood, double & min_distance);
    void GetLikelihoodMDN_MT(CompleteSpeedCurve ego_car_speed_curve, SpeedParam obstacle_car_speed_curve, double & likelihood, double & min_distance , MDN mdn_mt);
    void GetLikelihoodGMM(CompleteSpeedCurve ego_car_speed_curve, SpeedParam obstacle_car_speed_curve, double & likelihood, double & min_distance);
    void GetLikelihoodIRL(CompleteSpeedCurve ego_car_speed_curve, SpeedParam obstacle_car_speed_curve, double & likelihood, double & min_distance, MatrixXd & record_data);

    void GetLikelihoodGMM_groundtruth(Eigen::MatrixXd ego_groundtruth, SpeedParam obstacle_car_speed_curve, double & likelihood, MatrixXd & record_data);
    void GetLikelihoodIRL_groundtruth(Eigen::MatrixXd ego_groundtruth, SpeedParam obstacle_car_speed_curve, double & likelihood, MatrixXd & record_data);
    void GetLikelihoodMDN_MT_groundtruth(Eigen::MatrixXd ego_groundtruth, SpeedParam obstacle_car_speed_curve, double & likelihood, MatrixXd & record_data, MDN mdn_mt);
    void GetLikelihoodMDN_groundtruth(Eigen::MatrixXd ego_groundtruth, SpeedParam obstacle_car_speed_curve, double & likelihood, MatrixXd & record_data);

    // @ brief: input the minimum distance between two cars and get the cost
    // @ param: minimum distacne
    // @ output : the cost casued by minimum distance
    inline double DistanceToCost(double min_distance)
    {
        if(min_distance < 10.0)
        {
            return 1 / pow(min_distance, 2);
            //return 1 / pow(min_distance * 0.01, 2);
        }
        else
        {
            return 1 / pow(min_distance, 2);
        }

    }

    double SendDataToMDN(const deque<double>& ego_car_x_record, const deque<double>& ego_car_y_record, const deque<double>& obstacle_car_x_record, const deque<double>& obstacle_car_y_record, double next_ego_v, double next_obstacle_v);

    double ComputeObstacleCarDistance(double x, double y, double theta, double t,const vector<car_obstacle_>& car_obstacle);
    double ComputeObstacleCarDistance(double x0, double y0, double theta0, double x1, double y1, double theta1);

    double SendToGMM(vector<double> current_state, vector<double> next_velocity);
    inline void RecordToEigen(const deque<double>& ego_x_record, const deque<double>& ego_y_record, const deque<double>& obstacle_x_record, const deque<double>& obstacle_y_record, MatrixXd & result)
    {
        if(ego_x_record.size() != 5 || ego_y_record.size() != 5|| obstacle_x_record.size() != 5 || obstacle_y_record.size() != 5)
        {
            cout << "Error! the size you input into the MDN is wrong!\n";
        }
        result = Eigen::MatrixXd::Zero(5,4);
        result << ego_x_record[0], ego_y_record[0], obstacle_x_record[0], obstacle_y_record[0],
                ego_x_record[1], ego_y_record[1], obstacle_x_record[1], obstacle_y_record[1],
                ego_x_record[2], ego_y_record[2], obstacle_x_record[2], obstacle_y_record[2],
                ego_x_record[3], ego_y_record[3], obstacle_x_record[3], obstacle_y_record[3],
                ego_x_record[4], ego_y_record[4], obstacle_x_record[4], obstacle_y_record[4];
    }

    void GenerateLongTermForPlot(SpeedParam short_term_speed_param);
    void compute_future_distance(CompleteSpeedCurve ego_car_speed_curve, SpeedParam obstacle_car_speed_curve, double & min_distance);

    // void SendToGMM(vector<double> current_state, vector<double> next_velocity, double &this_step_likelihood);
};
