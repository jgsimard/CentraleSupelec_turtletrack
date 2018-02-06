#pragma once

#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

class KalmanFilter {

public:

  /**
  *   Parametres.
  *   A - Matrice de transition de l'etat
  *   B - Matrice de transition de la commande (pas implemente)
  *   C - Matrice de sortie de l'etat
  *   D - Matrice de sortie de la commande (pas implemente)
  *   Q - Matrice de la covariance du bruit de transition
  *   R - Matrice de la covariance du bruit de mesure
  *   P - Matrice d'estimation de la covariance de l'erreur
  */
  
  KalmanFilter(double dt,const Eigen::MatrixXd& A,const Eigen::MatrixXd& C,
               const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,const Eigen::MatrixXd& P);

  KalmanFilter() = delete;
  void init();
  void init(double t0, const Eigen::VectorXd& x0);
  void update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double dt_);
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };

private:

  Eigen::MatrixXd A, B, C, D, Q, R, P, K, P0, S;
  int m, n;                          // Dimensions
  double t0, t;
  double dt;
  bool initialized;
  Eigen::MatrixXd I;                 // Matrice Identite
  Eigen::VectorXd x_hat, x_hat_new ;  // Estimated states
  Eigen::VectorXd expected_value, innovation ; 

};
