#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */

  //state prediction
  x_ = F_ * x_;

  //predicted state estimate covariance:
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  // calculate innovation:
  VectorXd y = VectorXd(3);
  y = z - H_ * x_;

  MatrixXd S = MatrixXd(3, 3);
  MatrixXd Ht = MatrixXd(4, 3);
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;

  // Kalman gain:
  MatrixXd K = MatrixXd(4, 3);
  K = P_ * Ht * S.inverse();

  // Update state:
  x_ = x_ + K * y;

  // Update Covariance:
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // predicted measurement vector:
  VectorXd z_p = VectorXd(3);
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1) / rho, x_(0) / rho );
  float rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  z_p << rho, phi, rho_dot; 
  
  // calculate innovation:
  VectorXd y = VectorXd(3);
  y = z - z_p;
  //normalize y:
  y(1) = atan2(sin(y(1)), cos(y(1)));

  MatrixXd S = MatrixXd(3, 3);
  MatrixXd Ht = MatrixXd(4, 3);
  Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;

  // Kalman gain:
  MatrixXd K = MatrixXd(4, 3);
  K = P_ * Ht * S.inverse();

  // Update state:
  x_ = x_ + K * y;

  // Update Covariance:
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
