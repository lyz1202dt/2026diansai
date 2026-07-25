/**
 * @file OLED.c
 * @brief OLED显示屏I2C驱动程序实现
 * @details 基于SSD1306控制器的128x64像素OLED显示屏驱动实现
 *          支持字符、数字、字符串显示等功能
 * @version 1.0
 * @date 2025-07-12
 * @author TI Competition Team
 */

#include "OLED.h"
#include "OLED_Font.h"

/** @brief I2C通信超时时间（毫秒） */
#define I2C_TIMEOUT_MS (10)

#define OLED_ADDR      (0x3c)

/**
 * @brief I2C_OLED显存格式说明
 * @details 显存按页组织，共8页，每页128列
 * - [0]0 1 2 3 ... 127  (第0页)
 * - [1]0 1 2 3 ... 127  (第1页)
 * - [2]0 1 2 3 ... 127  (第2页)
 * - [3]0 1 2 3 ... 127  (第3页)
 * - [4]0 1 2 3 ... 127  (第4页)
 * - [5]0 1 2 3 ... 127  (第5页)
 * - [6]0 1 2 3 ... 127  (第6页)
 * - [7]0 1 2 3 ... 127  (第7页)
 */

/**
 * @brief 禁用I2C并配置为GPIO模式
 * @details 将I2C外设复位，然后重新配置SCL和SDA引脚为GPIO模式
 *          SCL配置为输出模式，SDA配置为输入模式
 * @return 始终返回0（成功）
 * @note 用于I2C总线恢复时的GPIO模拟，通常在总线卡死时调用
 */
static int mspm0_i2c_disable(void)
{
    DL_I2C_reset(I2C_OLED_INST);
    DL_GPIO_initDigitalOutput(GPIO_I2C_OLED_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_OLED_IOMUX_SDA,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_NONE,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
    return 0;
}

/**
 * @brief 启用I2C外设功能
 * @details 重新配置引脚为I2C功能并初始化I2C外设
 *          包括SCL和SDA引脚的功能配置、HiZ使能和I2C外设初始化
 * @return 始终返回0（成功）
 * @note 重新配置引脚为I2C功能并初始化I2C外设
 */
static int mspm0_i2c_enable(void)
{
    DL_I2C_reset(I2C_OLED_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SDA,
                                                GPIO_I2C_OLED_IOMUX_SDA_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SCL,
                                                GPIO_I2C_OLED_IOMUX_SCL_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SCL);
    DL_I2C_enablePower(I2C_OLED_INST);
    SYSCFG_DL_I2C_OLED_init();
    return 0;
}

/**
 * @brief I2C SDA线解锁函数
 * @details 当I2C总线卡死时，通过GPIO模拟产生时钟脉冲来释放SDA线
 *          最多尝试100次时钟脉冲，每次脉冲间隔1ms
 * @note 用于解决I2C总线死锁问题，在初始化时或通信异常时调用
 */
void I2C_OLED_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0;
    mspm0_i2c_disable();
    do
    {
        DL_GPIO_clearPins(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
        Delay(1);
        DL_GPIO_setPins(GPIO_I2C_OLED_SCL_PORT, GPIO_I2C_OLED_SCL_PIN);
        Delay(1);

        if (DL_GPIO_readPins(GPIO_I2C_OLED_SDA_PORT, GPIO_I2C_OLED_SDA_PIN)) break;
    } while (++cycleCnt < 100);
    mspm0_i2c_enable();
}

/**
 * @brief OLED显示颜色反转控制
 * @param i 反转控制参数
 *        - 0: 正常显示（黑底白字）
 *        - 1: 反色显示（白底黑字）
 * @note 通过发送SSD1306命令0xA6/0xA7控制显示颜色反转
 */
void I2C_OLED_ColorTurn(uint8_t i)
{
    if (i == 0)
    {
        I2C_OLED_WR_Byte(0xA6, I2C_OLED_CMD); // 正常显示
    }
    if (i == 1)
    {
        I2C_OLED_WR_Byte(0xA7, I2C_OLED_CMD); // 反色显示
    }
}

/**
 * @brief OLED屏幕旋转180度控制
 * @param i 旋转控制参数
 *        - 0: 正常显示方向
 *        - 1: 旋转180度显示
 * @note 通过发送SSD1306命令控制屏幕显示方向
 */
void I2C_OLED_DisplayTurn(uint8_t i)
{
    if (i == 0)
    {
        I2C_OLED_WR_Byte(0xC8, I2C_OLED_CMD); // 正常显示
        I2C_OLED_WR_Byte(0xA1, I2C_OLED_CMD);
    }
    if (i == 1)
    {
        I2C_OLED_WR_Byte(0xC0, I2C_OLED_CMD); // 反转显示
        I2C_OLED_WR_Byte(0xA0, I2C_OLED_CMD);
    }
}

/**
 * @brief 向SSD1306写入一个字节数据
 * @param dat 要写入的数据字节
 * @param mode 数据类型标志
 *        - 0: 表示命令数据
 *        - 1: 表示显示数据
 * @details 通过I2C总线向OLED控制器发送数据，包含超时处理机制
 *          如果通信超时，会自动调用SDA解锁函数恢复总线
 * @note I2C从机地址固定为0x3C
 */
void I2C_OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    unsigned char ptr[2];
    unsigned long start, cur;

    if (mode)
    {
        ptr[0] = 0x40;
        ptr[1] = dat;
    }
    else
    {
        ptr[0] = 0x00;
        ptr[1] = dat;
    }
    start = Sys_GetTick();

    DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, ptr, 2);
    DL_I2C_clearInterruptStatus(I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_OLED_INST, 0x3C, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    while (!DL_I2C_getRawInterruptStatus(I2C_OLED_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        cur = Sys_GetTick();
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            I2C_OLED_i2c_sda_unlock();
            break;
        }
    }
}

/**
 * @brief 设置OLED显示起始坐标
 * @param x 列坐标 (0-127)
 * @param y 页坐标 (0-7)
 * @details 设置SSD1306的页地址和列地址，为后续写入数据做准备
 *          页地址对应8个像素高度，列地址对应128个像素宽度
 * @note 坐标系统基于SSD1306的页模式寻址
 */
void I2C_OLED_Set_Pos(uint8_t x, uint8_t y)
{
    I2C_OLED_WR_Byte(0xb0 + (y / 8),
                     I2C_OLED_CMD); // y除以8得到页面号
    I2C_OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, I2C_OLED_CMD);
    I2C_OLED_WR_Byte((x & 0x0f), I2C_OLED_CMD);
}

/**
 * @brief 开启OLED显示
 * @details 启用SSD1306的电荷泵和显示功能
 *          发送命令序列: 0x8D, 0x14, 0xAF
 * @note 在初始化完成后或需要重新点亮屏幕时调用
 */
void I2C_OLED_Display_On(void)
{
    I2C_OLED_WR_Byte(0X8D, I2C_OLED_CMD); // SET DCDC命令
    I2C_OLED_WR_Byte(0X14, I2C_OLED_CMD); // DCDC ON
    I2C_OLED_WR_Byte(0XAF, I2C_OLED_CMD); // DISPLAY ON
}

/**
 * @brief 关闭OLED显示
 * @details 关闭SSD1306的电荷泵和显示功能以节省功耗
 *          发送命令序列: 0x8D, 0x10, 0xAE
 * @note 用于低功耗模式或临时关闭显示
 */
void I2C_OLED_Display_Off(void)
{
    I2C_OLED_WR_Byte(0X8D, I2C_OLED_CMD); // SET DCDC命令
    I2C_OLED_WR_Byte(0X10, I2C_OLED_CMD); // DCDC OFF
    I2C_OLED_WR_Byte(0XAE, I2C_OLED_CMD); // DISPLAY OFF
}

/**
 * @brief 清除OLED屏幕内容
 * @details 将整个128x64像素的显示区域清零，使屏幕显示为全黑
 *          通过遍历8个页面，每页128列的方式清除所有像素
 * @note 清屏后整个屏幕将显示为黑色背景
 */
void I2C_OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        I2C_OLED_WR_Byte(0xb0 + i, I2C_OLED_CMD); // 设置页地址（0~7）
        I2C_OLED_WR_Byte(0x00, I2C_OLED_CMD); // 设置显示位置—列低地址
        I2C_OLED_WR_Byte(0x10, I2C_OLED_CMD); // 设置显示位置—列高地址
        for (n = 0; n < 128; n++) I2C_OLED_WR_Byte(0, I2C_OLED_DATA);
    } // 更新显示
}

/**
 * @brief 在指定位置显示一个字符
 * @param x 起始列坐标 (0-127)
 * @param y 起始行坐标 (0-63)
 * @param sizey 字体大小
 *        - 8: 6x8像素字体
 *        - 16: 8x16像素字体
 * @param chr 要显示的字符
 * @details 根据字体大小从字体库中获取字符数据并显示到指定位置
 *          支持6x8和8x16两种字体大小
 * @note 字符显示基于ASCII码，字体数据存储在OLED_Font.h中
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t sizey, uint8_t chr)
{
    uint8_t c = 0, sizex = sizey / 2;
    uint16_t i = 0, size1;
    if (sizey == 8)
    {
        size1 = 6;
        c = chr - ' '; // 得到偏移后的值
        I2C_OLED_Set_Pos(x, y);
        for (i = 0; i < size1; i++)
        {
            I2C_OLED_WR_Byte(asc2_0806[c][i], I2C_OLED_DATA); // 6X8字号
        }
    }
    else if (sizey == 16)
    {
        size1 = 16; // 16字节数据
        c = chr - ' '; // 得到偏移后的值

        // 显示上半部分 (前8个字节)
        I2C_OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
        {
            I2C_OLED_WR_Byte(asc2_1608[c][i], I2C_OLED_DATA);
        }

        // 显示下半部分 (后8个字节)
        I2C_OLED_Set_Pos(x, y + 8);
        for (i = 8; i < 16; i++)
        {
            I2C_OLED_WR_Byte(asc2_1608[c][i], I2C_OLED_DATA);
        }
    }
}

/**
 * @brief 计算m的n次幂
 * @param m 底数
 * @param n 指数
 * @return m的n次幂结果
 * @note 用于数字显示时的位数计算
 */
uint32_t I2C_OLED_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief 在指定位置显示数字
 * @param x 起始列坐标
 * @param y 起始行坐标  
 * @param num 要显示的数字
 * @param sizey 字体大小 (8或16)
 * @param len 显示的位数长度
 * @details 将数字按指定位数显示，支持前导零消除
 *          自动计算每一位数字并调用字符显示函数
 * @note 数字超出指定位数时会截断高位
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t sizey, uint8_t len)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t char_width = (sizey == 8) ? 6 : 8; // 字符宽度

    for (t = 0; t < len; t++)
    {
        temp = (num / I2C_OLED_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + char_width * t, y, sizey, ' ');
                continue;
            }
            else enshow = 1;
        }
        OLED_ShowChar(x + char_width * t, y, sizey, temp + '0');
    }
}

/**
 * @brief 在指定位置显示字符串
 * @param x 起始列坐标
 * @param y 起始行坐标
 * @param sizey 字体大小 (8或16)
 * @param chr 指向要显示的字符串的指针
 * @details 逐个字符显示字符串，自动计算字符间距
 *          遇到'\0'结束符时停止显示
 * @note 字符串超出屏幕边界时不会自动换行
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t sizey, uint8_t *chr)
{
    uint8_t j = 0;
    uint8_t char_width = (sizey == 8) ? 6 : 8; // 字符宽度

    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, sizey, chr[j++]);
        x += char_width;
    }
}

/**
 * @brief 格式化输出到OLED屏幕
 * @param x 起始列坐标
 * @param y 起始行坐标
 * @param sizey 字体大小 (8或16)
 * @param format 格式化字符串，类似printf的格式
 * @param ... 可变参数列表
 * @return 格式化后的字符串长度
 * @details 支持printf风格的格式化输出，内部缓冲区大小为256字节
 *          使用vsprintf进行格式化处理
 * @note 类似于printf函数，但输出到OLED屏幕而非终端
 */
uint16_t OLED_Printf(uint8_t x, uint8_t y, uint8_t sizey, const char *format, ...)
{
    char Buffer[256];
    va_list args;
    va_start(args, format);
    uint16_t len = 0;
    len = vsprintf(Buffer, format, args);
    va_end(args);
    OLED_ShowString(x, y, sizey, Buffer);
}

/**
 * @brief 初始化SSD1306 OLED显示器
 * @details 执行完整的SSD1306初始化序列，包括：
 *          - 检查并解锁I2C总线
 *          - 配置显示参数（对比度、扫描方向等）
 *          - 设置时钟分频和预充电周期
 *          - 启用电荷泵和显示功能
 * @note 必须在使用OLED其他功能前调用此函数
 */
void OLED_Init(void)
{
    if (DL_I2C_getSDAStatus(I2C_OLED_INST) == DL_I2C_CONTROLLER_SDA_LOW) I2C_OLED_i2c_sda_unlock();

    Delay(200);

    I2C_OLED_WR_Byte(0xAE, I2C_OLED_CMD); //--turn off I2C_OLED panel
    I2C_OLED_WR_Byte(0x00, I2C_OLED_CMD); //---set low column address
    I2C_OLED_WR_Byte(0x10, I2C_OLED_CMD); //---set high column address
    I2C_OLED_WR_Byte(0x40, I2C_OLED_CMD); //--set start line address  Set Mapping RAM
    //Display Start Line (0x00~0x3F)
    I2C_OLED_WR_Byte(0x81, I2C_OLED_CMD); //--set contrast control register
    I2C_OLED_WR_Byte(0xCF, I2C_OLED_CMD); // Set SEG Output Current Brightness
    I2C_OLED_WR_Byte(0xA1,
                     I2C_OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    I2C_OLED_WR_Byte(0xC8,
                     I2C_OLED_CMD); // Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    I2C_OLED_WR_Byte(0xA6, I2C_OLED_CMD); //--set normal display
    I2C_OLED_WR_Byte(0xA8, I2C_OLED_CMD); //--set multiplex ratio(1 to 64)
    I2C_OLED_WR_Byte(0x3f, I2C_OLED_CMD); //--1/64 duty
    I2C_OLED_WR_Byte(0xD3,
                     I2C_OLED_CMD); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    I2C_OLED_WR_Byte(0x00, I2C_OLED_CMD); //-not offset
    I2C_OLED_WR_Byte(0xd5, I2C_OLED_CMD); //--set display clock divide ratio/oscillator frequency
    I2C_OLED_WR_Byte(0x80,
                     I2C_OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
    I2C_OLED_WR_Byte(0xD9, I2C_OLED_CMD); //--set pre-charge period
    I2C_OLED_WR_Byte(0xF1,
                     I2C_OLED_CMD); // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    I2C_OLED_WR_Byte(0xDA, I2C_OLED_CMD); //--set com pins hardware configuration
    I2C_OLED_WR_Byte(0x12, I2C_OLED_CMD);
    I2C_OLED_WR_Byte(0xDB, I2C_OLED_CMD); //--set vcomh
    I2C_OLED_WR_Byte(0x40, I2C_OLED_CMD); // Set VCOM Deselect Level
    I2C_OLED_WR_Byte(0x20, I2C_OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    I2C_OLED_WR_Byte(0x02, I2C_OLED_CMD); //
    I2C_OLED_WR_Byte(0x8D, I2C_OLED_CMD); //--set Charge Pump enable/disable
    I2C_OLED_WR_Byte(0x14, I2C_OLED_CMD); //--set(0x10) disable
    I2C_OLED_WR_Byte(0xA4, I2C_OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
    I2C_OLED_WR_Byte(0xA6, I2C_OLED_CMD); // Disable Inverse Display On (0xa6/a7)
    I2C_OLED_Clear();
    I2C_OLED_WR_Byte(0xAF, I2C_OLED_CMD); /*display ON*/
}
