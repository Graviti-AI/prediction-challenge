#ifndef CILQR_TYPE_DEF_H
#define CILQR_TYPE_DEF_H

#include <Eigen/Core>
#include <vector>

class ObstacleParameter
{
public:
	ObstacleParameter(const Eigen::MatrixXd& x0_, double half_L_ = 2.5, double half_W_ = 1.1);
	std::tuple<Eigen::Vector2d, double, double> D2Poly(Eigen::Vector2d p0, Eigen::MatrixXd poly);

	void makePolyPrediction(int N, double Ts); // generate Predicted trajectory of obstacle, assume constant speed
	std::vector<Eigen::MatrixXd> predicted_poly;	// poly for k=0 : N-1 --> position at kth sample time
	Eigen::MatrixXd poly; // 4 coordinates of an obstacle, the first row is X, then Y
	Eigen::MatrixXd traj_center; // the trajectory of the obstacle's rear center point

	Eigen::MatrixXd predicted_x; // for AStarSearch, [which_lane; s or index; v; theta] at each time step (column)
	// which_lane @ -1: far from lanes (point overlap), 0: left lane, 1: center lane, 2: right lane, 
	// 							only point overlap ---> 10: left and center lane, 21: center and right lane
	bool point_overlap;

	double half_L;   // half vehicle length
	double half_W;   // half vehicle width
	double max_acc, min_acc;

private:
	Eigen::VectorXd x0; // initial state, 6x1 Vector [x,y,theta,vx,vy,yaw_rate], (x,y) is the center of obstacle
};

struct CILQRParameter {
	// controller param
	int DIM_X;	// dim of x, [x, y, v, thata]
	int DIM_U;	// dim of u, [acc, curvature]
	int DIM_C;	// number of constraints per state vector 
	int max_iterate;
	double inter_threshold;
	double outer_threshold;

	size_t N;	// horizon length, including the initial state
	double Ts;	// sampling time

	double bt, max_bt; // Barrier function, 1/bt*log(-g(x))
	double barrier_scale;
	double bq1, bq2, max_bq1, max_bq2; // bq1*exp(bq2*g(x))
	double line_search_scale;
	Eigen::MatrixXd Q, Qf, R;

	// ego vehicle param
	Eigen::VectorXd x0; // initial state
	double L;	// ego car length
	double W;	// ego car width

	double max_steer_deg; // Max steer angle
	double max_lat_acc; // Max lateral acceleration 
	double max_long_acc, min_long_acc;

	double v_ref; // car speed reference 

	double margin_obs;	// margin for obstacles
	double margin_curb; // margin for road curb
};

class CAStarParameter 
{
public:
	// Search param
    int DIM_X;	// dim of x
	int DIM_U;	// dim of u
	size_t N;	// horizon length, including the initial state
	double Ts;	// sampling time

    int gap; // (CILQR.N-1) / (AStar.N-1) = gap
    double max_len_road; // max length of planned road
    double max_len_scale; // for refer_lane or lane_s lane_curv lane_width and list
    double max_v_scale; // for check_list and cost_list
    int scale_s, scale_v; // scale for judging if it is in open list
    int scale_lane_s; // for lane_curv, lane_s, lane_width
    Eigen::VectorXd actions; // discreted acceleration, eg. [1,0,-1,-2]mm/s^2
    Eigen::VectorXi change_lane_T; // discreted time for lane changing, eg. [10, 14, 20]*Ts(s), Ts = 0.2ms
    Eigen::MatrixXi first_sector_T; // discreted time for first sector, eg. [4, 5, 6; 6, 7, 8; 8, 10, 12]*Ts(s)
    double w_vref, w_acc, w_path_dev, w_curv_dev, w_jerk; // So far, we didn't use w_curv and w_jerk

	// ego vehicle param
	Eigen::VectorXd x0; // initial state, [x,y,v0,theta0]
	double L;	// ego car length
	double W;	// ego car width

	double max_steer_deg; // Max steer angle
	double max_lat_acc; // Max lateral acceleration 
	double max_long_acc, min_long_acc;

	double v_ref; // car speed reference 

	double margin_obs;	// margin for obstacles
	double margin_curb; // margin for road curb
};

class CNode
{
public:
	CNode() {
		this->s = 0;
		this->v = 0;
		this->tk = 0;
		this->lane = 1;
		this->path_cost = 0;
		this->heuristic_cost = 0;
		this->total_cost = 0;
	}
	CNode(const double s_, const double v_, const int time_, const int lane_, 
			const float path_cost_, const float heuristic_cost_, const float total_cost_) :
		s(s_),
		v(v_),
		tk(time_),
		lane(lane_),
		path_cost(path_cost_),
		heuristic_cost(heuristic_cost_),
		total_cost(total_cost_)
	{
	}

    double s, v;
	int tk, lane; // time = tk*Ts;
	std::vector<double> acc; // store curv and acc
	std::vector<double> curv;
	float path_cost;
	float heuristic_cost;
	float total_cost;
};

#endif