#ifndef __CONFIG_H__
#define __CONFIG_H__


#include "projdefs.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>

/* 使用 driverlib 的 DMA 接口 */
#include "Driver/uart/uart.h"
#include "Bsp/motor.h"
#include "Bsp/mpu6050.h"
#include "Bsp/gw_model.h"
#include "Bsp/OLED.h"
#include "Lib/pid/PID.h"


extern L298N_t g_l298n;
extern Encoder_t g_encoder1,g_encoder2;
extern PID wheel1_vel_pid,wheel2_vel_pid;
extern SerialHandle_t *g_serial;
extern SerialHandle_t *zdt_serial;

void SetupConfig();


#endif