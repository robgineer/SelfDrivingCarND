#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// set parameters to values from arguments
	// access class members via this (required as argument names are identical to class members)
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	// set initial errors to 0 (expecting a correct initial state because the car is driving straight at the beginning)
	this->p_error = 0;
	this->i_error = 0;
	this->d_error = 0;

}

void PID::UpdateError(double cte) {

	// differential error represents the difference of two consecutive error values
	this->d_error = cte - this->p_error;
	// proportional error represents the "cross track error" that is taken over from the simulator
	this->p_error = cte;
	// integral error represents the sum of all previous errors (long term memory)
	this->i_error = this->i_error + cte;
}

double PID::TotalError() {

	// return the sum of all errors (== total error)
	return this->Kp * this->p_error + this->Ki * this->i_error + this->Kd * this->d_error;
}

