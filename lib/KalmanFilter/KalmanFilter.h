#pragma once
#include "BasicLinearAlgebra.h"
using namespace BLA;

class KalmanFilter
{
private:
    double Kp = 0.5;
    double Kd = 0.5;
    double Kspeed = 1;
    Matrix<3, 1, double> x = {0, 0, 0};
    Matrix<3, 1, double> B = {0, 0, 1};
    Matrix<3, 3, double> P = Zeros<3, 3, double>();

    Matrix<3, 3, double> F;
    Matrix<1, 1, double> y;
    Matrix<1, 1, double> z;
    Matrix<3, 1, double> K;

    const Matrix<1, 3, double> H = {0, 0, 1};

    const Matrix<3, 3, double> I = {1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1};

    const Matrix<3, 3, double> Q = {2, 3, 3,
                                    3, 1, 1,
                                    3, 1, 1};
    const Matrix<1, 1, double> R = {1};

public:
    KalmanFilter() = default;
    ~KalmanFilter() = default;

    double filter(double accel, double pwm, double dt);
    void reset();
};