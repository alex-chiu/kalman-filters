#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    // Constructor
    KalmanFilter(Eigen::Vector3f initialPos);

private:
    Eigen::Vector3f pos;
};

#endif