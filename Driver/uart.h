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

/* 回调函数类型定义 */
typedef void (*ErrorCb)(int err_code, void* param);
typedef void (*RecvCb)(uint8_t *data, uint16_t size, void* param);

/* 串口句柄结构体 */
typedef struct{
    UART_Regs *hardware;
    uint8_t *send_buffer;
    uint8_t *recv_buffer;
    uint16_t send_buffer_size;
    uint16_t recv_buffer_size;
    uint16_t tx_length;          /* 本次发送长度 */
    uint16_t tx_index;           /* 当前发送索引 */
    uint16_t recv_count;        /* 当前接收计数 */
    uint16_t recv_expected;      /* 本次接收目标长度 */
    SerialMode_t mode;
    ErrorCb error_cb;
    RecvCb recv_cb;
    void* param;
    SemaphoreHandle_t rx_sem;   /* 接收完成信号量 */
    SemaphoreHandle_t tx_sem;   /* 发送完成信号量 */
    uint8_t dma_rx_ch;          /* DMA 接收通道 */
    uint8_t dma_tx_ch;          /* DMA 发送通道 */
    volatile bool rx_wait_idle;
    volatile bool rx_using_dma;
    volatile bool rx_done;
    volatile bool tx_done;
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
 */
int SerialTransmit(SerialHandle_t* handle, uint8_t *data, uint16_t size);

/**
 * @brief 接收定长数据 (流式接收)
 * @param handle 串口句柄
 * @param data 接收缓冲区
 * @param size 要接收的长度
 * @param timeout 超时时间 (ms)，-1 表示无限等待
 * @return 实际接收的字节数，或错误码
 */
int SerialReceive(SerialHandle_t* handle, uint8_t *data, uint16_t size, int timeout);

/**
 * @brief 接收不定长数据 (由硬件空闲事件判定一帧结束)
 * @param handle 串口句柄
 * @param data 接收缓冲区
 * @param max_size 接收缓冲区最大长度
 * @return 实际接收的字节数，或错误码
 */
int SerialReceiveIDLE(SerialHandle_t* handle, uint8_t *data, uint16_t max_size);

/**
 * @brief UART 中断处理函数 (需要在 UART ISR 中调用)
 * @param handle 串口句柄
 */
void SerialIRQ(SerialHandle_t* handle);

/**
 * @brief 配置 DMA 通道 (仅在 DMA 模式下需要调用)
 * @param handle 串口句柄
 * @param dma_tx_ch DMA TX 通道号
 * @param dma_rx_ch DMA RX 通道号
 * @return 错误码
 */
int SerialConfigDMA(SerialHandle_t* handle, uint8_t dma_tx_ch, uint8_t dma_rx_ch);

/**
 * @brief 反初始化串口，释放资源
 * @param handle 串口句柄
 * @return 错误码
 */
int SerialDeinit(SerialHandle_t* handle);

#endif
