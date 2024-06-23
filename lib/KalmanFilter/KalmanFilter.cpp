#include "KalmanFilter.h"
using namespace BLA;

double KalmanFilter::filter(double accel, double pwm, double dt)
{
    // predict moment
    double u = (1 / dt) * (Kp * (Kspeed * pwm - static_cast<double>(x(1, 0))) - Kd * static_cast<double>(x(2, 0)));
    F = {1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1};

    x = F * x + B * u;
    P = F * P * (~P) + Q;

    // update moment
    z = {accel};
    y = z - H * x;
    K = P * (~H) * (~((H * P) * (~H) + R));

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
