#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <math.h>

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/*
 * Sensor independent prediction
 * */
void KalmanFilter::Predict() {

	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}
/**
 * Laser specific  update step
 */
void KalmanFilter::Update(const VectorXd &z) {
	// predicted state
	VectorXd z_predicted = H_ * x_;
	// error
	VectorXd y = z -  z_predicted;
	// H_ transpose
	MatrixXd Ht = H_.transpose();
	// System uncertainty
	MatrixXd S = H_ * P_ * Ht + R_;
	//Kalman gain
	MatrixXd K = P_ * Ht * S.inverse();
	// new estimation
	x_ = x_ + K * y;

	// uncertainty covariance of estimation
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

/**
 * Radar specific update step
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float rho_pre = px*px+py*py;

	// check division by zero
	if(fabs(rho_pre) < 0.0001)
	{
		std::cout << "UpdateEKF () - potential division by 0" << std::endl;
		rho_pre = 0.0001;
	}

	// h(x)
	float rho 		= sqrtf(rho_pre);
	float phi		= atan2(py,px);
	float rho_dot 	= (px*vx+py*vy)/rho;

	VectorXd z_predicted = VectorXd(3);
	z_predicted << rho, phi, rho_dot;

	// error
	VectorXd y = z -  z_predicted;

	// correcting phi values if out of range
	if(y(1) > M_PI)
	{
		while(y(1) > M_PI)
		{
			y(1) = y(1) - 2*M_PI;
		}
	}
	else if(y(1) < - M_PI)
	{
		while(y(1) < - M_PI)
		{
			y(1) = y(1) + 2*M_PI;
		}
	}

	// H_ transpose
	MatrixXd Ht = H_.transpose();
	// System uncertainty
	MatrixXd S = H_ * P_ * Ht + R_;
	// Kalman gain
	MatrixXd K = P_ * Ht * S.inverse();
	// new estimation
	x_ = x_ + K * y;

	// uncertainty covariance of estimation
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

