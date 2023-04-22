#include "KALMAN.h"

Kalman::Kalman()
{
    qAngle = 0.001f;
    qBias = 0.003f;
    rMeasure = 0.03f;

    angle = 0.0f;
    gyroBias = 0.0f;

    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// Angle: Degrees
// Rate:  Degrees / Second
// dT:    Seconds
float Kalman::getAngle(float newAngle, float newRate, float dt)
{
    /* Step 1 */
    rate = newRate - gyroBias;
    angle += dt * rate;

    /* Step 2 */
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + qAngle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += qBias * dt;

    /* Step 3 */
    float S = P[0][0] + rMeasure; // Estimate error

    /* Step 4 */
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    /* Step 5 */
    float y = newAngle - angle;

    /* Step 6 */
    angle += K[0] * y;
    gyroBias += K[1] * y;

    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; };
float Kalman::getRate() { return this->rate; };

void Kalman::setQAngle(float qAngle) { this->qAngle = qAngle; };
void Kalman::setQBias(float qBias) { this->qBias = qBias; };
void Kalman::setRMeasure(float R_measure) { this->rMeasure = R_measure; };

float Kalman::getQAngle() { return this->qAngle; };
float Kalman::getQBias() { return this->qBias; };
float Kalman::getRMeasure() { return this->rMeasure; };
