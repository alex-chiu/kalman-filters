#include "KalmanFilter.h"
#include <Eigen/Dense>
#include <iostream>

// Constructor
KalmanFilter::KalmanFilter(Eigen::Vector3f initialPos,
                           Eigen::Vector3f initialVel) {
  KalmanFilter::pos = initialPos;
  KalmanFilter::vel = initialVel;
  std::cout << "Filter initialized with initial position: " << initialPos
            << std::endl;
}
