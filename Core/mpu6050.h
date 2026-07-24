#ifndef __MPU6050_H
#define __MPU6050_H

#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>

#define MPU6050_TIMEOUT      1000

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48

#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_PWR_MGMT_2   0x6C
#define MPU6050_WHO_AM_I     0x75

void mpu6050_i2c_sda_unlock(void);
int mspm0_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
int mspm0_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

extern short gyro[3], accel[3];
extern float pitch, roll, yaw;

void MPU6050_Init(void);
int read_quad(float *q);
void get_euler_angles(const float *q,float *roll,float *pitch,float *yaw);

#endif