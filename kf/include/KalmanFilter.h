#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
  // Constructor
  KalmanFilter(Eigen::Vector3f initialPos, Eigen::Vector3f initialVel);

private:
  Eigen::Vector3f pos;
  Eigen::Vector3f vel;
};

#endif