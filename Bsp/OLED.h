/*
 * SysConfig Configuration Steps:
 *   I2C:
 *     1. Add an I2C module.
 *     2. Name it as "OLED_I2C".
 *     3. Check the box "Enable Controller Mode".
 *     4. Set "Standard Bus Speed" to "Fast Mode (400kHz)". (optional)
 *     5. Set the pins according to your needs.
 */

#ifndef __I2C_OLED_H__
#define __I2C_OLED_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define I2C_OLED_CMD  0 //写命令
#define I2C_OLED_DATA 1 //写数据

void I2C_OLED_ColorTurn(uint8_t i);
void I2C_OLED_DisplayTurn(uint8_t i);
void I2C_OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void I2C_OLED_Set_Pos(uint8_t x, uint8_t y);
void I2C_OLED_Display_On(void);
void I2C_OLED_Display_Off(void);
void I2C_OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t sizey, uint8_t chr);
uint32_t I2C_OLED_pow(uint8_t m, uint8_t n);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t sizey, uint8_t len);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t sizey, uint8_t *chr);
uint16_t OLED_Printf(uint8_t x, uint8_t y, uint8_t sizey, const char *format, ...);
void OLED_Init(void);
void I2C_OLED_i2c_sda_unlock(void);

#endif /* #ifndef __I2C_OLED_H__ */
