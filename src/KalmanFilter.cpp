#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter( double dt,const Eigen::MatrixXd& A,const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,const Eigen::MatrixXd& P)
  : A(A), B(B),  C(C), D(D), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double dt_) {
  dt = dt_;
  
  //prediction step
  x_hat_new = A * x_hat + B * u;
  P = A * P * A.transpose() + Q;

  //Observation step
  expected_value = C * x_hat_new + D * u;
  S = C * P * C.transpose() + R;
  K = P * C.transpose() * S.inverse();
  innovation = y - expected_value;
  x_hat_new += K * innovation;
  P = (I - K * C) * P;
  x_hat = x_hat_new;

  t += dt;
}
