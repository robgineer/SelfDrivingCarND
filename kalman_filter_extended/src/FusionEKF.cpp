#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {

  // set initialization to false upon creation of new instance
  is_initialized_ = false;

  // initialize timestamp
  previous_timestamp_ = 0;

  /**
   * initialize matrices
   */

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // measurement transition matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

  // Jacobian matrix Hj_
  Hj_ = MatrixXd(3, 4);
  Hj_ << 1, 1, 0, 0,
	     1, 1, 0, 0,
	     1, 1, 1, 1;

  //state transition matrix F_
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
		  	 0, 1, 0, 0,
			 0, 0, 1000, 0,
			 0, 0, 0, 1000;

  // set acceleration noise (x and y direction)
   noise_ax = 9;
   noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {
    // initialize state vector
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      /**
      Convert radar from polar to cartesian coordinates
      */
    		float rho = measurement_pack.raw_measurements_[0];
    		float phi = measurement_pack.raw_measurements_[1];

    		float px = rho * cos(phi);
    		float py = rho * sin(phi);

    		//rho_dot not required according to project intro

    		// update state vector with first values
    		ekf_.x_ << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
    		/** this is simple as no conversion necessary. Set the velocities to 0
    	    	as these represent the hidden part (that is to be observed) **/
    	    	ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // store first timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // calculate dynamic values for state transition
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) /* Radar updates */
  {
	  // return; // testing radar only

	  // set measurement function to Jacobian matrix
	  Hj_ = tools.CalculateJacobian(ekf_.x_);;
	  ekf_.H_ = Hj_;
	  // set the sensor specific covariance
	  ekf_.R_ = R_radar_;
	  // update with non linear inputs
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else /* Laser updates */
  {
	  // return; // testing lidar only

	  // set laser specific measurement function
	  ekf_.H_ = H_laser_;
	  // set the sensor specific covariance
	  ekf_.R_ = R_laser_;
	  // update with linear inputs
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
