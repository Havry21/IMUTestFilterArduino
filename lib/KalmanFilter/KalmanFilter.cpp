#include "KalmanFilter.h"
using namespace BLA;

double KalmanFilter::filter(double accel, double pwm, double dt)
{
    // Serial.print("dt - ");
    // Serial.println(dt);
    // predict moment

    // u - mm/sec
    // double dt = _dt / 1000;
    double u = (Kp * (Kspeed * pwm - static_cast<double>(x(1, 0))) - Kd * static_cast<double>(x(2, 0)) * dt);
    // Serial.print("F - ");
    // F.printTo(Serial);
    F = {1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1};
    // Serial.println();

    x = F * x + B * u;

    P = F * P * (~F) + Q;

    // update moment
    // m/s2 to -/*  */mm/s2
    z = {-accel / 1000};
    y = z - H * x;
    Matrix<1, 1, double> invS;

    Invert(~((H * P) * (~H) + R), invS);
    // Serial.print("InvS - ");
    // invS.printTo(Serial);
    // Serial.println();

    // Serial.print("K - ");
    K = P * (~H) * invS;
    // K.printTo(Serial);
    // Serial.println();
    x = x + K * y;
    P = (I - K * H) * P;

    return x(0, 0);
};

void KalmanFilter::reset()
{
    x.Fill(0);
    B.Fill(0);
    F.Fill(0);
    P.Fill(0);
    y.Fill(0);
    z.Fill(0);
    K.Fill(0);
}
