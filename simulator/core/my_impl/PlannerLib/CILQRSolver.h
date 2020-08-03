#pragma once
#include <vector>
#include <Eigen/Core>
#include "CILQRTypeDef.h"

struct Point {
	double x, y;
};

class CILQRSolver
{
public:
	CILQRSolver(CILQRParameter param_);
	~CILQRSolver();

	std::vector<Point> road_info, refer_path;
	Eigen::MatrixXd x_ref, u_ref;	// reference along time
	Eigen::MatrixXd u_init; // initial iteration trajectory
	
	void addObstacle(const ObstacleParameter & obs_);
	void initSetup();
	void run();
	Eigen::MatrixXd getOptimalState();
	Eigen::MatrixXd getOptimalControl();
	
	void forwardSimulation(const Eigen::MatrixXd& u_sim, Eigen::MatrixXd& x);
	void createInputConstraint(); 
	void createStateConstraint(const Eigen::MatrixXd& x);
	decltype(auto) getHx(int i, int k);
	void setMode(int mode_);
	void computeStateCostJacobian(int k, const Eigen::MatrixXd& x, double& phix, Eigen::MatrixXd& dphix, Eigen::MatrixXd& ddphix);
	void computeInputCostJacobian(int k, const Eigen::MatrixXd& u, double& phiu, Eigen::MatrixXd& dphiu, Eigen::MatrixXd& ddphiu);
	void computeStateTransMatrix(const Eigen::VectorXd& xk, const Eigen::VectorXd& uk, Eigen::MatrixXd& A, Eigen::MatrixXd& B);
	bool computeCost(double& cost, const Eigen::MatrixXd& u, const Eigen::MatrixXd & x, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Qf, const Eigen::MatrixXd& R);

private:
	size_t n_obs;
	std::vector<ObstacleParameter> obstacles;
	Eigen::MatrixXd Hx, mx; // state constraints : given by createStateConstraints()
	
	CILQRParameter prob_param; // param of problem
	Eigen::MatrixXd Hu, mu; // input constraints : given by createInputConstraints()

	double bq1, bq2, bt; // barrier function parameters
	int mode; // mode = 0 means 1/t*log(-g(x)), mode = 1 means bq1*exp(bq2*g(x)) (default)
	Eigen::MatrixXd optimal_x, optimal_u;
};
