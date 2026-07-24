#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} GPIO_Pin_t;

typedef struct {
    // L298N INx方向控制引脚
    GPIO_Pin_t in1;
    GPIO_Pin_t in2;

    // L298N ENA/ENB PWM输出
    DL_TIMER_CC_INDEX pwm_cc_index;

    // 运行状态/校准参数
    bool invert_direction;
    uint16_t duty_permille;  // 0~1000, 便于做占空比限幅
} L298N_Channel_t;

typedef struct {
    // PWM使用的定时器
    GPTIMER_Regs* pwm_timer;
    // 双路H桥，对应OUT1/OUT2和OUT3/OUT4
    L298N_Channel_t motor1;
    L298N_Channel_t motor2;
}L298N_t;


typedef struct{
    GPIO_Pin_t cha;
    GPIO_Pin_t chb;
    volatile int32_t encoder_value;
}Encoder_t;     //基于外部中断的编码器


//L298N硬件资源初始化
void L298N_Init(L298N_t*handle);
//L298N设置电机占空比
void L298N_SetPWMValue(L298N_t*handle,uint16_t channel,int32_t pwm_value);

void  EncoderInit(Encoder_t*handle);
//编码器值更新（在中断中调用）
bool EncoderUpdate(Encoder_t *handle, GPIO_Regs *interrupt_port, uint32_t interrupt_pin);

#endif
