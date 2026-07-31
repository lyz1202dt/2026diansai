#include "kalman.h"

#define KALMAN_MIN_NOISE  (1.0e-9f)

static float Kalman1D_LimitPositive(float value) {
    if (value < KALMAN_MIN_NOISE) {
        return KALMAN_MIN_NOISE;
    }

    return value;
}

void Kalman1D_Init(Kalman1D *filter, float position, float velocity, float q, float r, float p) {
    if (filter == 0) {
        return;
    }

    filter->position = position;
    filter->velocity = velocity;
    filter->q = Kalman1D_LimitPositive(q);
    filter->r = Kalman1D_LimitPositive(r);

    p = Kalman1D_LimitPositive(p);
    filter->p[0][0] = p;
    filter->p[0][1] = 0.0f;
    filter->p[1][0] = 0.0f;
    filter->p[1][1] = p;
}

void Kalman1D_SetNoise(Kalman1D *filter, float q, float r) {
    if (filter == 0) {
        return;
    }

    filter->q = Kalman1D_LimitPositive(q);
    filter->r = Kalman1D_LimitPositive(r);
}

void Kalman1D_Update(Kalman1D *filter, float measured_position, float dt) {
    float dt2;
    float predicted_position;
    float predicted_velocity;
    float p00;
    float p01;
    float p10;
    float p11;
    float innovation;
    float innovation_covariance;
    float k0;
    float k1;

    if ((filter == 0) || (dt <= 0.0f)) {
        return;
    }

    dt2 = dt * dt;

    predicted_position = filter->position + filter->velocity * dt;
    predicted_velocity = filter->velocity;

    p00 = filter->p[0][0] + dt * (filter->p[1][0] + filter->p[0][1]) + dt2 * filter->p[1][1] +
          filter->q * dt2;
    p01 = filter->p[0][1] + dt * filter->p[1][1];
    p10 = filter->p[1][0] + dt * filter->p[1][1];
    p11 = filter->p[1][1] + filter->q;

    innovation = measured_position - predicted_position;
    innovation_covariance = Kalman1D_LimitPositive(p00 + filter->r);
    k0 = p00 / innovation_covariance;
    k1 = p10 / innovation_covariance;

    filter->position = predicted_position + k0 * innovation;
    filter->velocity = predicted_velocity + k1 * innovation;

    filter->p[0][0] = (1.0f - k0) * p00;
    filter->p[0][1] = (1.0f - k0) * p01;
    filter->p[1][0] = p10 - k1 * p00;
    filter->p[1][1] = p11 - k1 * p01;
}
