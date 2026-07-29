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

void Kalman1D_Update(Kalman1D *filter, float measured_position, float acceleration, float dt) {
    float dt2;
    float dt3;
    float dt4;
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
    dt3 = dt2 * dt;
    dt4 = dt2 * dt2;

    predicted_position = filter->position + filter->velocity * dt + 0.5f * acceleration * dt2;
    predicted_velocity = filter->velocity + acceleration * dt;

    p00 = filter->p[0][0] + dt * (filter->p[1][0] + filter->p[0][1]) + dt2 * filter->p[1][1] +
          0.25f * filter->q * dt4;
    p01 = filter->p[0][1] + dt * filter->p[1][1] + 0.5f * filter->q * dt3;
    p10 = filter->p[1][0] + dt * filter->p[1][1] + 0.5f * filter->q * dt3;
    p11 = filter->p[1][1] + filter->q * dt2;

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
