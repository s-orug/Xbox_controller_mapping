#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman
{
public:
    Kalman();

    // Sets the starting angle
    void setAngle(float angle);

    // These functions are used to tune the Kalman filter
    void setQAngle(float variance);
    void setQBias(float variance);
    void setRMeasure(float variance);

    // Calculates and returns the angle in degrees, given newAngle (in degrees), newRate (in degrees per second), and delta time (in seconds)
    float getAngle(float newAngle, float newRate, float deltaTime);

    // Returns the unbiased rate
    float getRate();

    // Getters for processNoiseVarianceForAccelerometer, processNoiseVarianceForGyroBias, and measurementNoiseVariance
    float getQAngle();
    float getQBias();
    float getRMeasure();

private:
    // Kalman filter variables
    float qAngle;
    float qBias;
    float rMeasure;

    float angle;    // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float gyroBias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;     // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

#endif
