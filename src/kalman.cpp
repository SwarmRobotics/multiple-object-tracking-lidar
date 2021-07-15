/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kf_tracker/kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter(int n_, int m_, double dt_) {
    m=m_;
    n=n_;
    A= Eigen::MatrixXd::Zero(n,n);
    C=Eigen::MatrixXd::Zero(m,n);
    Q=Eigen::MatrixXd::Zero(n,n);
    R=Eigen::MatrixXd::Zero(m,m);
    P0=Eigen::MatrixXd::Zero(n,n);
    dt=dt_;
    initialized=false;
    I=Eigen::MatrixXd::Zero(n,n);
    I.setIdentity();
    x_hat= Eigen::VectorXd::Zero(n);
    x_hat_new= Eigen::VectorXd::Zero(n);

    for(int i=0; i<n; i++)
    {
      A(i,i)=1;
      if(i<n-1)
        A(i,i+1)=dt;
      Q(i,i)=0.01;
      P0(i,i)=0.01;
    }
    for(int i=0; i<m; i++)
    {
      R(i,i)=0.01;
    }
    for(int i=0; i<m; i++)
    {
      for(int j=0; j<n; j++)
      {
        if (i==j)
         C(i,j)=1;
      }
    }

}

void KalmanFilter::init(double t0, const Eigen::VectorXd x0) {
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

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  // std::cout<<"y " << y <<std::endl
  // << " P"<< P<<std::endl
  // << " A"<< A<<std::endl
  // << " K"<< K<<std::endl
  // << " Q"<< Q<<std::endl
  // << "x_hat old "<<x_hat<<std::endl
  // << "x_hat new "<<x_hat_new<<std::endl;

  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}
