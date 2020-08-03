#include "PidController.h"
#include <stdexcept>
#include <cmath>
using namespace std;

PidController::PidController(double tspan)
{
	t_delta = 0.0;
	this->tspan = tspan;
}


PidController::~PidController()
{
}
bool PidController::finish()
{

	//cout << c1 << "	" << c2 << endl;
	if (x>90)
		return true;
	else
		return false;
};
void PidController::locupdate_temporary()
{
	count = 0;
	double dotpro = (x_map[count + 1] - x)*(x_map[count + 1] - x_map[count]) + (y_map[count + 1] - y)*(y_map[count + 1] - y_map[count]);
	while ((count < (int)(x_map.size() - 1)) && (dotpro) < 0)
	{
		count++;
		dotpro = (x_map[count + 1] - x)*(x_map[count + 1] - x_map[count]) + (y_map[count + 1] - y)*(y_map[count + 1] - y_map[count]);
	}
	x0 = x_map[count];
	y0 = y_map[count];
	x1 = x_map[count + 1];
	y1 = y_map[count + 1];
	l = pow(pow(x1 - x0, 2) + pow(y1 - y0, 2), 0.5);
	theta1 = Yaw-atan2(y1 - y0, x1 - x0);
};
void PidController::locupdate_prediction()
{
	count = 0;
	double dotpro = (x_map[count + 1] - x_nxt)*(x_map[count + 1] - x_map[count]) + (y_map[count + 1] - y_nxt)*(y_map[count + 1] - y_map[count]);
	while ((count < (int)(x_map.size()-1)) && (dotpro) < 0)
	{
		count++;
		dotpro = (x_map[count + 1] - x_nxt)*(x_map[count + 1] - x_map[count]) + (y_map[count + 1] - y_nxt)*(y_map[count + 1] - y_map[count]);
	}
	x0 = x_map[count];
	y0 = y_map[count];
	x1 = x_map[count + 1];
	y1 = y_map[count + 1];
	l = pow(pow(x1 - x0, 2) + pow(y1 - y0, 2), 0.5);
	s_pre = s_frenet;
	s_frenet = ((x0 - x_nxt)*(y1 - y0) - (x1 - x0)*(y0 - y_nxt)) / l;
	double temp = (x0 - x_nxt)*(x1 - x0) + (y0 - y_nxt)*(y1 - y0);
	a = pow(pow(x0 - x_nxt, 2) + pow(y0 - y_nxt, 2), 0.5);
	if (temp > 0)
	{
		u_frenet =- pow(pow(a, 2) - pow(s_frenet, 2), 0.5);
	}
	else
	{
		u_frenet = -pow(pow(a, 2) - pow(s_frenet, 2), 0.5);
	}
}

void PidController::update(double state[6])
{
	x = state[0];
	y = state[1];
	Yaw = state[2];
	if (Yaw > PI)
		Yaw -= 2 * PI;
	else if (Yaw < -PI)
		Yaw += 2 * PI;
	else;
	v = pow(pow(state[3], 2) + pow(state[4], 2), 0.5);
	x_nxt = x + state[3] * t_delta;
	y_nxt = y + state[4] * t_delta;
	phi = atan2(state[4], state[3])-Yaw;
	dYawdt = state[5];
}
void PidController::PIDcontrol(double tspan)
{
	S_integral += tspan * s_frenet;
	S_integral = fmin(fmax(S_integral, 2), -2);
	
	output_a = (v_plan[count+1] - v)*0.5;
	
	if (abs(output_a) > 5)
	{
		output_a = abs(output_a) / output_a * 5;
	}
	output_theta = -2*tspan*s_frenet;
	if (abs(output_theta) > PI / 4)
	{
		output_theta = abs(output_theta) / output_theta * PI / 4;
	}
	
	/*
	output_a = (v_plan[count + 1] - v)*0.5;

	if (abs(output_a) > 5)
	{
		output_a = abs(output_a) / output_a * 5;
	}
	output_a /= 5;
	output_theta = -0.8 * tspan*s_frenet-2*(s_frenet-s_pre);
	if (abs(output_theta) > 1)
	{
		output_theta = abs(output_theta) / output_theta * 1;
	}
	*/
}

Vector PidController::internalUpdate(const Vector & currState, const Vector & planResult)
{
	double state[6];
	state[0] = currState[0];
	state[1] = currState[1];
	state[2] = currState[2];
	state[3] = currState[3];
	state[4] = currState[4];
	state[5] = currState[5];

	this->update(state);

	//

	const int PLAN_STEP = 10;
	if (planResult.size() != 4 * PLAN_STEP) {
		throw std::runtime_error("bad length of planning result");
	}

	this->t = Vector(PLAN_STEP);
	this->x_map = Vector(PLAN_STEP);
	this->y_map = Vector(PLAN_STEP);
	this->v_plan = Vector(PLAN_STEP);

	for (int i = 0; i < PLAN_STEP; i++) {
		this->t[i] = planResult[4 * i];
		this->x_map[i] = planResult[4 * i + 1];
		this->y_map[i] = planResult[4 * i + 2];
		this->v_plan[i] = planResult[4 * i + 3];
	}

	this->locupdate_temporary();
	this->locupdate_prediction();

	this->PIDcontrol(this->tspan);

	return Vector();
}


