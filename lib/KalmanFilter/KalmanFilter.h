#pragma once

class KalmanFilter
{
private:
    double Q; // Process noise covariance
    double R; // Measurement noise
    // covariance

public:
    double x; // Estimated value
    double P; // Estimated error
    double K = 0;
    KalmanFilter(double processNoise, double measurementNoise,
                 double initialValue, double initialError)
    {
        Q = processNoise;
        R = measurementNoise;
        x = initialValue;
        P = initialError;
    }
    double filter(double measurement)
    { // Prediction update
        double x_pred = x;
        double P_pred = P + Q; // Measurement update
        K = P_pred / (P_pred + R);
        x = x_pred + K * (measurement - x_pred);
        P = (1 - K) * P_pred;
        return x;
    }
};