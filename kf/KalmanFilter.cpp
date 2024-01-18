#include "KalmanFilter.h"
#include <iostream>
#include <Eigen/Dense>

// Constructor
KalmanFilter::KalmanFilter(Eigen::Vector3f initialPos) : pos(initialPos) {
    std::cout << "Filter initialized with initial position: " << initialPos << std::endl;
}
