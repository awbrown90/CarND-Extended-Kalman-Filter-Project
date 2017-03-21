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

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  
  R_radar_ << .09, 0, 0,
    		  0, .0009, 0,
    		  0, 0, .09;

  R_laser_ << 0.0255, 0,
    		0, 0.0255;

  H_laser_ << 1, 0, 0 ,0,
      		0, 1, 0, 0;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
   			 0, 1, 0, 1,
   			 0, 0, 1, 0,
   			 0, 0, 0, 1;

  ekf_.P_ = MatrixXd(4,4);

  //set the acceleration noise components
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
  if (!is_initialized_) {
    
    
    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    	//cout << "Radar " << endl;
      	ekf_.x_(0) = measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1));
	  	ekf_.x_(1) = measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1));
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    	//cout << "Laser " << endl;
    	ekf_.x_(0) = measurement_pack.raw_measurements_(0);
	    ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			 0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			 dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			 0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  //predict
  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    float x = ekf_.x_(0);
    float y = ekf_.x_(1);
    float vx = ekf_.x_(2);
    float vy = ekf_.x_(3);

    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    float rho = sqrt(x*x+y*y);
    float theta = atan2(y,x);
    float ro_dot = (x*vx+y*vy)/rho;
    VectorXd z_pred = VectorXd(3);
    z_pred << rho,theta,ro_dot;
    ekf_.UpdateRadar(measurement_pack.raw_measurements_,z_pred);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.UpdateLaser(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}




