#include "KalmanFilter.h"
#include <iostream>
#include <Eigen/Dense>

int main() {
    std::cout << "Initializing basic Kalman filter test" <<  std::endl;

    Eigen::VectorXf initialPos(3);
    initialPos << 5.0, 10.0, 15.0;

    KalmanFilter KF(initialPos);
}