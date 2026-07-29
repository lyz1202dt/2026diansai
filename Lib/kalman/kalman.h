#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一维位置/速度线性卡尔曼滤波器
 *
 * 状态量为 [position, velocity]，输入为 acceleration，测量量为 position。
 */
typedef struct {
    float position;     //!<@brief 估计位置
    float velocity;     //!<@brief 估计速度
    float q;            //!<@brief 过程噪声Q，表示加速度模型不确定度
    float r;            //!<@brief 测量噪声R，表示位置测量不确定度
    float p[2][2];      //!<@brief 估计误差协方差矩阵
} Kalman1D;

/**
 * @brief 初始化一维卡尔曼滤波器
 * @param[in] filter 滤波器结构体
 * @param[in] position 初始位置
 * @param[in] velocity 初始速度
 * @param[in] q 过程噪声Q
 * @param[in] r 测量噪声R
 * @param[in] p 初始估计误差协方差
 */
void Kalman1D_Init(Kalman1D *filter, float position, float velocity, float q, float r, float p);

/**
 * @brief 运行时调整Q和R参数
 * @param[in] filter 滤波器结构体
 * @param[in] q 过程噪声Q
 * @param[in] r 测量噪声R
 */
void Kalman1D_SetNoise(Kalman1D *filter, float q, float r);

/**
 * @brief 执行一次一维卡尔曼滤波
 * @param[in] filter 滤波器结构体
 * @param[in] measured_position 当前测量位置
 * @param[in] acceleration 当前加速度输入
 * @param[in] dt 采样周期，单位秒
 */
void Kalman1D_Update(Kalman1D *filter, float measured_position, float acceleration, float dt);

#ifdef __cplusplus
}
#endif

#endif
