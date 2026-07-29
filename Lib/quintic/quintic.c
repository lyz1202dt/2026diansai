#include "quintic.h"

void QuinticGenerate(Quintic *quintic, float start_pos, float stop_pos, float stop_vel, float time) {
    float delta;
    float end_vel_time;
    float time2;
    float time3;
    float time4;
    float time5;

    if (quintic == 0) {
        return;
    }

    quintic->start_pos = start_pos;
    quintic->stop_pos = stop_pos;
    quintic->stop_vel = stop_vel;
    quintic->total_time = time;

    quintic->a0 = start_pos;
    quintic->a1 = 0.0f;
    quintic->a2 = 0.0f;
    quintic->a3 = 0.0f;
    quintic->a4 = 0.0f;
    quintic->a5 = 0.0f;

    if (time <= 0.0f) {
        quintic->a0 = stop_pos;
        quintic->total_time = 0.0f;
        return;
    }

    delta = stop_pos - start_pos;
    end_vel_time = stop_vel * time;
    time2 = time * time;
    time3 = time2 * time;
    time4 = time3 * time;
    time5 = time4 * time;

    quintic->a3 = (10.0f * delta - 4.0f * end_vel_time) / time3;
    quintic->a4 = (-15.0f * delta + 7.0f * end_vel_time) / time4;
    quintic->a5 = (6.0f * delta - 3.0f * end_vel_time) / time5;
}

bool QuinticSample(float time, float *pos, float *vel, float *acc, Quintic *quintic) {
    float sample_time;
    float position;
    float velocity;
    float acceleration;
    bool is_finished;

    if (quintic == 0) {
        if (pos != 0) {
            *pos = 0.0f;
        }
        if (vel != 0) {
            *vel = 0.0f;
        }
        if (acc != 0) {
            *acc = 0.0f;
        }
        return true;
    }

    if (quintic->total_time <= 0.0f) {
        if (pos != 0) {
            *pos = quintic->stop_pos;
        }
        if (vel != 0) {
            *vel = quintic->stop_vel;
        }
        if (acc != 0) {
            *acc = 0.0f;
        }
        return true;
    }

    if (time <= 0.0f) {
        sample_time = 0.0f;
        is_finished = false;
    } else if (time >= quintic->total_time) {
        sample_time = quintic->total_time;
        is_finished = true;
    } else {
        sample_time = time;
        is_finished = false;
    }

    position = (((quintic->a5 * sample_time + quintic->a4) * sample_time + quintic->a3) * sample_time +
                quintic->a2) *
                       sample_time * sample_time +
               quintic->a1 * sample_time + quintic->a0;
    velocity = ((5.0f * quintic->a5 * sample_time + 4.0f * quintic->a4) * sample_time +
                3.0f * quintic->a3) *
                       sample_time * sample_time +
               2.0f * quintic->a2 * sample_time + quintic->a1;
    acceleration = ((20.0f * quintic->a5 * sample_time + 12.0f * quintic->a4) * sample_time +
                    6.0f * quintic->a3) *
                           sample_time +
                   2.0f * quintic->a2;

    if (is_finished) {
        position = quintic->stop_pos;
        velocity = quintic->stop_vel;
        acceleration = 0.0f;
    }

    if (pos != 0) {
        *pos = position;
    }
    if (vel != 0) {
        *vel = velocity;
    }
    if (acc != 0) {
        *acc = acceleration;
    }

    return is_finished;
}
