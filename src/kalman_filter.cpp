#include "kalman_filter.h"

using namespace std;

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

void KalmanFilter::Predict() {
   x_ = F_*x_;
   MatrixXd Ft = F_.transpose();
   P_ = F_*P_*Ft+Q_;
}
void KalmanFilter::UpdateRadar(const VectorXd &z, const VectorXd &z_pred) {
  
   VectorXd y = z - z_pred;

   //angle normalization
   while (y(1)> M_PI) y(1)-=2.*M_PI;
   while (y(1)<-M_PI) y(1)+=2.*M_PI;

   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_*P_*Ht + R_;

   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd k = PHt * Si;
   x_ = x_ + (k*y);

   long xsize = x_.size();
   MatrixXd I = MatrixXd::Identity(xsize,xsize);
   P_ = (I-k*H_)*P_;

}


void KalmanFilter::UpdateLaser(const VectorXd &z) {
  
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;

   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_*P_*Ht + R_;

   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_*Ht;
   MatrixXd k = PHt*Si;
   x_ = x_ + (k*y);

   long xsize = x_.size();
   MatrixXd I = MatrixXd::Identity(xsize,xsize);
   P_ = (I-k*H_)*P_;

}
