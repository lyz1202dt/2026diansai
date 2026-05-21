# DMA 发送使用指南

## 概述
本驱动库现已支持 DMA 发送模式。DMA 模式可以减少 CPU 负担，提高传输效率，特别是在发送大量数据时。

## 工作原理
- **DMA 模式发送**：数据被复制到发送缓冲区，然后由 DMA 硬件自动传输到 UART TX FIFO
- **中断处理**：通过两个中断标志
  - `DL_UART_MAIN_IIDX_DMA_DONE_TX`：DMA 传输完成
  - `DL_UART_MAIN_IIDX_EOT_DONE`：UART 发送完成（所有数据已从 FIFO 发出）

## 使用步骤

### 1. 初始化串口（DMA 模式）
```c
#include "uart.h"
#include <ti/driverlib/driverlib.h>

/* 创建串口句柄 */
SerialHandle_t* uart_handle = SerialInit(
    UART0_INST,           /* UART 硬件指针 */
    SERIAL_MODE_DMA,      /* DMA 模式 */
    NULL,                 /* 错误回调函数 */
    NULL                  /* 用户参数 */
);

if (!uart_handle) {
    /* 初始化失败 */
    return;
}
```

### 2. 配置 DMA 通道
```c
/* 配置 DMA 通道（须与 SysConfig 配置一致） */
int ret = SerialConfigDMA(
    uart_handle,
    DMA_CH0_CHAN_ID,    /* DMA TX 通道 */
    DMA_CH1_CHAN_ID     /* DMA RX 通道（如果需要） */
);

if (ret != SERIAL_OK) {
    /* 配置失败 */
    return;
}
```

### 3. 发送数据
```c
uint8_t data[] = "Hello DMA!";
int bytes_sent = SerialTransmit(uart_handle, data, sizeof(data) - 1);

if (bytes_sent > 0) {
    /* 数据已复制到缓冲区，DMA 开始传输 */
}
```

### 4. 在 UART 中断处理程序中调用驱动 IRQ 处理
```c
void UART0_IRQHandler(void)
{
    SerialIRQ(uart_handle);
}
```

## 完整示例

```c
#include "uart.h"
#include <ti/driverlib/driverlib.h>

SerialHandle_t* uart_handle = NULL;

void init_uart_dma(void)
{
    /* 1. 初始化串口为 DMA 模式 */
    uart_handle = SerialInit(UART0_INST, SERIAL_MODE_DMA, NULL, NULL);
    
    /* 2. 配置 DMA 通道 */
    if (uart_handle) {
        SerialConfigDMA(uart_handle, DMA_CH0_CHAN_ID, DMA_CH1_CHAN_ID);
        
        /* 3. 启用 UART 中断 */
        NVIC_EnableIRQ(UART0_INST_INT_IRQN);
    }
}

void send_data_via_dma(uint8_t* data, uint16_t length)
{
    if (uart_handle) {
        int bytes = SerialTransmit(uart_handle, data, length);
        if (bytes > 0) {
            /* 发送已启动 */
        } else {
            /* 发送失败 */
        }
    }
}

void UART0_IRQHandler(void)
{
    if (uart_handle) {
        SerialIRQ(uart_handle);
    }
}

int main(void)
{
    SYSCFG_DL_init();  /* 系统和外设初始化 */
    
    init_uart_dma();   /* 初始化 UART DMA */
    
    uint8_t test_data[] = "MSP!";
    send_data_via_dma(test_data, sizeof(test_data) - 1);
    
    while (1) {
        /* 主程序循环 */
        __WFI();  /* 等待中断 */
    }
}
```

## 中断处理流程

```
UART ISR 触发
    ↓
SerialIRQ() 检查状态
    ├→ DL_UART_MAIN_IIDX_DMA_DONE_TX：DMA 传输完成
    │   └→ 设置 tx_done = true
    │   └→ 释放发送完成信号量（如果配置了）
    │
    └→ DL_UART_MAIN_IIDX_EOT_DONE：UART 发送完成
        └→ 所有数据已从 FIFO 发出
```

## 缓冲区大小

- **发送缓冲区**：默认 256 字节
- **接收缓冲区**：默认 512 字节

如需更改，可在初始化后修改句柄的 `send_buffer_size` 和 `recv_buffer_size` 字段。

## 注意事项

1. **必须在 SysConfig 中配置 DMA 通道**
   - DMA 源地址指向发送缓冲区
   - DMA 目标地址指向 UART TX FIFO
   - 启用 DMA 中断

2. **DMA 通道号必须与 SysConfig 一致**
   - 通常在 `ti_msp_dl_config.h` 中定义为 `DMA_CH0_CHAN_ID` 等

3. **数据限制**
   - 发送数据大小不能超过 `send_buffer_size`（默认 256 字节）
   - 单次传输超过限制会被截断

4. **中断必须启用**
   - 必须在 UART ISR 中调用 `SerialIRQ()`
   - 否则无法检测发送完成

5. **FreeRTOS 支持**
   - 在 FreeRTOS 环境下自动使用二值信号量进行同步
   - 在裸机环境下使用轮询或其他同步机制

## 故障排查

| 问题 | 原因 | 解决方案 |
|-----|------|--------|
| 数据未发送 | DMA 通道未启用 | 检查 SerialConfigDMA() 是否被调用 |
| 发送超时 | DMA 中断未触发 | 确保 UART ISR 已注册，SerialIRQ() 已调用 |
| 数据不完整 | 缓冲区溢出 | 检查发送数据大小是否超过 256 字节 |
| 程序卡住 | 同步信号量问题 | 检查 FreeRTOS 配置是否正确 |
