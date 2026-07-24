#ifndef __OLED_H__
#define __OLED_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define OLED_CMD  0U
#define OLED_DATA 1U

void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat, uint8_t mode);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t sizey, uint8_t chr);
uint32_t OLED_pow(uint8_t m, uint8_t n);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t sizey, uint8_t len);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t sizey, const char *chr);
uint16_t OLED_Printf(uint8_t x, uint8_t y, uint8_t sizey, const char *format, ...);
void OLED_Init(void);
void I2C_OLED_i2c_sda_unlock(void);

/* Compatibility with the old I2C OLED API names. */
#define I2C_OLED_CMD         OLED_CMD
#define I2C_OLED_DATA        OLED_DATA
#define I2C_OLED_ColorTurn   OLED_ColorTurn
#define I2C_OLED_DisplayTurn OLED_DisplayTurn
#define I2C_OLED_WR_Byte     OLED_WR_Byte
#define I2C_OLED_Set_Pos     OLED_Set_Pos
#define I2C_OLED_Display_On  OLED_Display_On
#define I2C_OLED_Display_Off OLED_Display_Off
#define I2C_OLED_Clear       OLED_Clear
#define I2C_OLED_pow         OLED_pow

#endif /* __OLED_H__ */
