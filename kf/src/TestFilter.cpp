#include "KalmanFilter.h"
#include <Eigen/Dense>
#include <iostream>

int main() {
  std::cout << "### INITIALIZING BASIC KALMAN FILTER TEST ###" << std::endl;

  Eigen::VectorXf initialPos(3);
  initialPos << 5.0, 10.0, 15.0;

  Eigen::VectorXf initialVel(3);
  initialPos << 1.0, 2.0, 3.0;

  KalmanFilter KF(initialPos, initialVel);
}