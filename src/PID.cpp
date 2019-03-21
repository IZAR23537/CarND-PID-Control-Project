#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

void PID::UpdateError(double cte) {

	d_error = cte - p_error; // How much are we changed from the last cte, p_error in here is the previous cte.
	p_error = cte; // How far are we from the center.
	i_error += cte; // Sums of the CTEs.
}

double PID::TotalError() {

  return (Kp * p_error) + (Ki * i_error) + (Kd * d_error);
}

