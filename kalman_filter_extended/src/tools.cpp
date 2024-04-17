#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	//declare and initialize rmse vector
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	if((estimations.size() != ground_truth.size() )  || (estimations.size() == 0))
	{
		cout << "# of estimations does not correspond to # of ground truth samples " << endl;
		return rmse;
	}

	for(int i=0; i < estimations.size(); i++)
	{
		VectorXd res = estimations[i] - ground_truth[i];
		res = res.array()*res.array();
		rmse = rmse + res;
	}

	//mean value
	rmse = rmse/estimations.size();
	//squared root
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	// get values for px, py, vx, vy
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// define denominator variables
	float d1		= px*px+py*py;
	float d2		= sqrt(d1);
	float d3		= d1 * d2;

	//check division by zero
	if(fabs(d1) < 0.0001)
	{
		cout << "CalculateJacobian () - potential division by 0" << endl;
		d1 = 0.0001;
	}

	// declare matrix
	MatrixXd Jacobian = MatrixXd(3,4);

	// assign partial derivatives
	Jacobian << px/d2, py/d2, 0, 0,
			  -(py/d1), px/d1, 0, 0,
			  (py*(vx*py-vy*px))/d3, (px*(vy*px-vx*py))/d3, px/d2, py/d2;

	return Jacobian;
}
