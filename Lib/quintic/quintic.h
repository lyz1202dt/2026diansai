#ifndef __QUINTIC_H__
#define __QUINTIC_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 五次多项式轨迹参数
 *
 * 轨迹形式为:
 * position(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 *
 * 当前生成函数假设起点速度/加速度为0，终点加速度为0。
 */
typedef struct {
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    float a5;
    float start_pos;
    float stop_pos;
    float stop_vel;
    float total_time;
} Quintic;

/**
 * @brief 根据起末位置和终点速度生成五次多项式轨迹参数
 * @param[in,out] quintic 轨迹参数结构体
 * @param[in] start_pos 起点位置
 * @param[in] stop_pos 终点位置
 * @param[in] stop_vel 终点速度
 * @param[in] time 轨迹总时间，单位与采样时间一致
 */
void QuinticGenerate(Quintic *quintic ,float start_pos,float stop_pos,float stop_vel,float time);

/**
 * @brief 采样五次多项式轨迹
 * @param[in] time 当前轨迹时间
 * @param[out] pos 当前时刻位置，可传入0表示不需要
 * @param[out] vel 当前时刻速度，可传入0表示不需要
 * @param[out] acc 当前时刻加速度，可传入0表示不需要
 * @param[in] quintic 轨迹参数结构体
 * @return true 表示采样时间已达到或超过轨迹总时间，输出钳制为终点位置、终点速度和0加速度
 */
bool QuinticSample(float time,float* pos,float* vel,float* acc,Quintic *quintic);

#ifdef __cplusplus
}
#endif

#endif
