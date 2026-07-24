#include "OLED.h"

#include "OLED_Font.h"
#include <FreeRTOS.h>
#include <stdarg.h>
#include <stdio.h>
#include <task.h>

#define OLED_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static void OLED_DC_Write(uint8_t data_mode)
{
    if (data_mode != 0U)
    {
        DL_GPIO_setPins(OLED_CTRL_PORT, OLED_CTRL_OLED_DC_PIN);
    }
    else
    {
        DL_GPIO_clearPins(OLED_CTRL_PORT, OLED_CTRL_OLED_DC_PIN);
    }
}

static void OLED_Reset(void)
{
    DL_GPIO_setPins(OLED_CTRL_PORT, OLED_CTRL_OLED_RES_PIN);
    OLED_DELAY_MS(10);
    DL_GPIO_clearPins(OLED_CTRL_PORT, OLED_CTRL_OLED_RES_PIN);
    OLED_DELAY_MS(10);
    DL_GPIO_setPins(OLED_CTRL_PORT, OLED_CTRL_OLED_RES_PIN);
    OLED_DELAY_MS(100);
}

void I2C_OLED_i2c_sda_unlock(void)
{
    OLED_Reset();
}

void OLED_ColorTurn(uint8_t i)
{
    if (i == 0U)
    {
        OLED_WR_Byte(0xA6, OLED_CMD);
    }
    else if (i == 1U)
    {
        OLED_WR_Byte(0xA7, OLED_CMD);
    }
}

void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0U)
    {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    else if (i == 1U)
    {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    OLED_DC_Write(mode);
    DL_SPI_transmitDataBlocking8(OLED_SPI_INST, dat);
    while (DL_SPI_isBusy(OLED_SPI_INST))
    {
    }
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte((uint8_t) (0xB0U + (y / 8U)), OLED_CMD);
    OLED_WR_Byte((uint8_t) (((x & 0xF0U) >> 4U) | 0x10U), OLED_CMD);
    OLED_WR_Byte((uint8_t) (x & 0x0FU), OLED_CMD);
}

void OLED_Display_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_Clear(void)
{
    uint8_t i;
    uint8_t n;

    for (i = 0U; i < 8U; i++)
    {
        OLED_WR_Byte((uint8_t) (0xB0U + i), OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (n = 0U; n < 128U; n++)
        {
            OLED_WR_Byte(0x00, OLED_DATA);
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t sizey, uint8_t chr)
{
    uint8_t c;
    uint16_t i;

    if ((chr < ' ') || (chr > '~'))
    {
        chr = ' ';
    }

    c = (uint8_t) (chr - ' ');

    if (sizey == 8U)
    {
        OLED_Set_Pos(x, y);
        for (i = 0U; i < 6U; i++)
        {
            OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);
        }
    }
    else if (sizey == 16U)
    {
        OLED_Set_Pos(x, y);
        for (i = 0U; i < 8U; i++)
        {
            OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);
        }

        OLED_Set_Pos(x, (uint8_t) (y + 8U));
        for (i = 8U; i < 16U; i++)
        {
            OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);
        }
    }
}

uint32_t OLED_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1U;

    while (n-- != 0U)
    {
        result *= m;
    }

    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t sizey, uint8_t len)
{
    uint8_t t;
    uint8_t temp;
    uint8_t enshow = 0U;
    uint8_t char_width = (sizey == 8U) ? 6U : 8U;

    for (t = 0U; t < len; t++)
    {
        temp = (uint8_t) ((num / OLED_pow(10U, (uint8_t) (len - t - 1U))) % 10U);
        if ((enshow == 0U) && (t < (len - 1U)))
        {
            if (temp == 0U)
            {
                OLED_ShowChar((uint8_t) (x + char_width * t), y, sizey, ' ');
                continue;
            }
            enshow = 1U;
        }
        OLED_ShowChar((uint8_t) (x + char_width * t), y, sizey, (uint8_t) (temp + '0'));
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t sizey, const char *chr)
{
    uint8_t j = 0U;
    uint8_t char_width = (sizey == 8U) ? 6U : 8U;

    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, sizey, (uint8_t) chr[j++]);
        x = (uint8_t) (x + char_width);
    }
}

uint16_t OLED_Printf(uint8_t x, uint8_t y, uint8_t sizey, const char *format, ...)
{
    char buffer[128];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0)
    {
        return 0U;
    }

    OLED_ShowString(x, y, sizey, buffer);
    return (uint16_t) len;
}

void OLED_Init(void)
{
    OLED_Reset();

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);
}
