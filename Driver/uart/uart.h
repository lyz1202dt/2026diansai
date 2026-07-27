#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <ti/devices/msp/peripherals/hw_uart.h>
#include <ti/driverlib/dl_dma.h>

/* UART 工作模式 */
typedef enum {
    SERIAL_MODE_IT = 0,    /* 中断模式 */
    SERIAL_MODE_DMA = 1,   /* DMA 模式 */
    SERIAL_MODE_POLL = 2   /* 轮询模式 */
} SerialMode_t;

/* 错误码定义 */
typedef enum {
    SERIAL_OK = 0,
    SERIAL_ERR_TIMEOUT = -1,
    SERIAL_ERR_INVALID = -2,
    SERIAL_ERR_BUSY = -3,
    SERIAL_ERR_ALLOC = -4
} SerialError_t;

/* HAL 风格状态定义 */
typedef enum {
    SERIAL_STATE_RESET = 0,
    SERIAL_STATE_READY = 1,
    SERIAL_STATE_BUSY  = 2
} SerialState_t;

/* 回调函数类型定义 */
typedef void (*ErrorCb)(int err_code, void* param);

/* 串口句柄结构体 */
typedef struct{
    UART_Regs *hardware;
    uint8_t *send_buffer;
    uint8_t *recv_buffer;
    uint16_t send_buffer_size;
    uint16_t recv_buffer_size;
    uint16_t tx_length;          /* 本次发送长度 */
    volatile uint16_t tx_index;  /* 当前发送索引 */
    volatile uint16_t recv_count;/* 当前接收计数 */
    uint16_t recv_expected;      /* 本次接收目标长度 */

    SerialMode_t mode;           /* 当前工作模式 */
    volatile SerialState_t tx_state; /* 发送状态 */
    volatile SerialState_t rx_state; /* 接收状态 */
    SemaphoreHandle_t tx_sem;    /* 发送完成信号量 */
    SemaphoreHandle_t rx_sem;    /* 接收完成信号量 */
    ErrorCb error_cb;            /* 错误回调 */
    void *param;                 /* 用户参数 */
} SerialHandle_t;

/**
 * @brief 初始化串口
 * @param hw_uart UART 硬件指针
 * @param mode 工作模式 (IT/DMA/POLL)
 * @param error_cb 错误回调函数
 * @param param 用户参数
 * @return 串口句柄指针
 */
SerialHandle_t* SerialInit(UART_Regs *hw_uart, uint8_t mode, ErrorCb error_cb, void* param);

/**
 * @brief 串口发送数据
 * @param handle 串口句柄
 * @param data 发送数据指针
 * @param size 发送数据长度
 * @return 发送的字节数，或错误码
 * @note 函数会阻塞直到发送全部完成，在发送完成中断中使用二值信号量唤醒后函数返回
 */
int SerialTransmit(SerialHandle_t* handle, uint8_t *data, uint16_t size);

/**
 * @brief 接收定长数据 (流式接收)
 * @param handle 串口句柄
 * @param data 接收缓冲区
 * @param size 要接收的长度
 * @param timeout 超时时间 (ms)，-1 表示无限等待
 * @return 实际接收的字节数，或错误码
 * @note 函数会阻塞直到接收到指定个数的数字，或者接收超时，在接收完成中断中使用二值信号量唤醒后函数返回。或者提前因为超时返回
 */
int SerialReceive(SerialHandle_t* handle, uint8_t *data, uint16_t size,int timeout);

/**
 * @brief UART 中断处理函数 (需要在 UART ISR 中调用)
 * @param handle 串口句柄
 */
void SerialIRQ(SerialHandle_t* handle);

/**
 * @brief 反初始化串口，释放资源
 * @param handle 串口句柄
 * @return 错误码
 */
int SerialDeinit(SerialHandle_t* handle);

#endif
