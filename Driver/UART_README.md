# 可复用串口库 (Reusable UART Library)

## 概述

这是一个为 MSPM0G3507 微控制器设计的轻量级、可复用串口驱动库，支持以下特性：

- **多种工作模式**：轮询 (POLL)、中断 (IT)、DMA 模式
- **FreeRTOS 集成**：自动适配 FreeRTOS 环境，支持信号量同步
- **动态内存管理**：自动分配/释放缓冲区
- **灵活的回调机制**：错误回调、接收完成回调
- **定长/不定长接收**：支持多种数据接收方式

---

## 核心结构体

### SerialHandle_t

```c
typedef struct{
    UART_Regs *hardware;          // UART 硬件指针
    uint8_t *send_buffer;         // 发送缓冲区
    uint8_t *recv_buffer;         // 接收缓冲区
    uint16_t send_buffer_size;    // 发送缓冲区大小
    uint16_t recv_buffer_size;    // 接收缓冲区大小
    uint16_t recv_count;          // 当前接收计数
    SerialMode_t mode;            // 工作模式
    ErrorCb error_cb;             // 错误回调
    RecvCb recv_cb;               // 接收完成回调
    void* param;                  // 用户参数
    SemaphoreHandle_t rx_sem;     // 接收信号量
    SemaphoreHandle_t tx_sem;     // 发送信号量
    DMA_Channel_t dma_rx_ch;      // DMA 接收通道
    DMA_Channel_t dma_tx_ch;      // DMA 发送通道
    volatile bool rx_done;        // 接收完成标志
    volatile bool tx_done;        // 发送完成标志
} SerialHandle_t;
```

---

## API 参考

### 初始化和反初始化

#### SerialInit()
```c
SerialHandle_t* SerialInit(UART_Regs *hw_uart, uint8_t mode, 
                           ErrorCb error_cb, void* param);
```
**功能**：初始化串口
- **参数**：
  - `hw_uart`: UART 硬件指针 (如 `UART_0_INST`)
  - `mode`: 工作模式 (`SERIAL_MODE_POLL`/`SERIAL_MODE_IT`/`SERIAL_MODE_DMA`)
  - `error_cb`: 错误回调函数指针
  - `param`: 用户自定义参数
- **返回值**：串口句柄指针，失败返回 NULL

**示例**：
```c
SerialHandle_t *handle = SerialInit(UART_0_INST, SERIAL_MODE_IT, 
                                    my_error_callback, NULL);
```

#### SerialDeinit()
```c
int SerialDeinit(SerialHandle_t* handle);
```
**功能**：反初始化串口，释放资源
- **返回值**：`SERIAL_OK` 或错误码

---

### 数据发送

#### SerialTransmit()
```c
int SerialTransmit(SerialHandle_t* handle, uint8_t *data, uint16_t size);
```
**功能**：发送数据

| 工作模式 | 行为 |
|---------|------|
| POLL | 直接轮询发送 |
| IT | 使用中断发送 |
| DMA | 返回错误 (需单独配置) |

- **参数**：
  - `handle`: 串口句柄
  - `data`: 待发送数据
  - `size`: 数据长度
- **返回值**：发送的字节数或错误码

**示例**：
```c
uint8_t msg[] = "Hello\n";
int result = SerialTransmit(handle, msg, sizeof(msg) - 1);
if (result > 0) {
    // 发送成功
}
```

---

### 数据接收

#### SerialReceive() - 定长接收
```c
int SerialReceive(SerialHandle_t* handle, uint8_t *data, 
                  uint16_t size, int timeout);
```
**功能**：接收指定长度的数据

| 工作模式 | 行为 |
|---------|------|
| POLL | 轮询接收直到达到 size 或超时 |
| IT/DMA | 等待 size 个字节后返回 |

- **参数**：
  - `handle`: 串口句柄
  - `data`: 接收缓冲区
  - `size`: 要接收的长度
  - `timeout`: 超时时间 (ms)，-1 表示无限等待
- **返回值**：接收的字节数或错误码

**示例**：
```c
uint8_t buffer[64];
int result = SerialReceive(handle, buffer, 64, 1000);  // 超时 1 秒
if (result > 0) {
    printf("Received %d bytes\n", result);
}
```

#### SerialReceiveIDLE() - 不定长接收
```c
int SerialReceiveIDLE(SerialHandle_t* handle, uint8_t *data, 
                      uint16_t max_size, int timeout);
```
**功能**：接收不定长数据（直到遇到 `\r` 或 `\n`，或超时）

- **参数**：
  - `handle`: 串口句柄
  - `data`: 接收缓冲区
  - `max_size`: 缓冲区最大长度
  - `timeout`: 超时时间 (ms)
- **返回值**：实际接收的字节数或错误码

**示例**：
```c
uint8_t cmd[128];
int size = SerialReceiveIDLE(handle, cmd, sizeof(cmd), 500);
if (size > 0) {
    // 处理接收到的命令
    process_command(cmd, size);
}
```

---

### 中断处理

#### SerialIRQ()
```c
void SerialIRQ(SerialHandle_t* handle);
```
**功能**：UART 中断处理函数

**使用方法**：在 UART ISR 中调用

```c
void UART0_IRQHandler(void)
{
    SerialIRQ(g_serial_handle);
}
```

---

## 工作模式详解

### 1. 轮询模式 (SERIAL_MODE_POLL)
```c
SerialHandle_t *handle = SerialInit(UART_0_INST, SERIAL_MODE_POLL, NULL, NULL);

// 发送：直接轮询发送
SerialTransmit(handle, data, 10);

// 接收：轮询等待直到收到指定长度数据或超时
SerialReceive(handle, buffer, 64, 1000);
```

**优点**：
- 实现简单，无需中断配置
- 确定性强

**缺点**：
- CPU 利用率低
- 不适合实时性要求高的应用

---

### 2. 中断模式 (SERIAL_MODE_IT)
```c
SerialHandle_t *handle = SerialInit(UART_0_INST, SERIAL_MODE_IT, 
                                    error_callback, NULL);

// 在 UART ISR 中
void UART0_IRQHandler(void)
{
    SerialIRQ(handle);
}

// 发送：启用发送中断
SerialTransmit(handle, data, 10);

// 接收：等待信号量
SerialReceive(handle, buffer, 64, 1000);
```

**优点**：
- CPU 利用率高
- 响应及时

**缺点**：
- 需要中断配置
- 缓冲区大小固定

---

### 3. DMA 模式 (SERIAL_MODE_DMA)

**注意**：DMA 模式需要在应用层单独配置 DMA 寄存器和中断。库会自动处理 DMA 完成中断。

```c
SerialHandle_t *handle = SerialInit(UART_0_INST, SERIAL_MODE_DMA, NULL, NULL);

// 应用层需要配置 DMA：
// 1. 设置 DMA 源地址：UART_0_INST->RXDATA
// 2. 设置 DMA 目的地址：handle->recv_buffer
// 3. 设置传输大小
// 4. 启用 DMA 通道和中断

// 库会在 DMA 完成时自动触发信号量
```

---

## 错误码

| 错误码 | 值 | 说明 |
|--------|-----|------|
| `SERIAL_OK` | 0 | 成功 |
| `SERIAL_ERR_TIMEOUT` | -1 | 超时 |
| `SERIAL_ERR_INVALID` | -2 | 无效参数 |
| `SERIAL_ERR_BUSY` | -3 | 忙碌/缓冲区满 |
| `SERIAL_ERR_ALLOC` | -4 | 内存分配失败 |

---

## 回调函数

### 错误回调
```c
void my_error_callback(int err_code, void* param)
{
    switch (err_code) {
        case SERIAL_ERR_TIMEOUT:
            printf("UART Timeout\n");
            break;
        case SERIAL_ERR_BUSY:
            printf("UART Buffer Full\n");
            break;
        default:
            break;
    }
}
```

### 接收完成回调
```c
void my_recv_callback(uint8_t *data, uint16_t size, void* param)
{
    printf("Received %d bytes: ", size);
    for (int i = 0; i < size; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}
```

---

## 完整应用示例

### 场景：主机通过串口接收命令并回复

```c
#include "uart.h"
#include <FreeRTOS.h>
#include <task.h>

SerialHandle_t *g_uart = NULL;

void error_callback(int err, void *p)
{
    if (err == SERIAL_ERR_TIMEOUT) {
        printf("UART Rx Timeout\n");
    }
}

void recv_callback(uint8_t *data, uint16_t size, void *p)
{
    printf("Received command: ");
    for (int i = 0; i < size; i++) printf("%c", data[i]);
    printf("\n");
}

void uart_task(void *param)
{
    uint8_t cmd[64];
    int size;

    while (1) {
        // 等待接收命令 (带 1000ms 超时)
        size = SerialReceiveIDLE(g_uart, cmd, sizeof(cmd), 1000);

        if (size > 0) {
            // 回复确认
            SerialTransmit(g_uart, cmd, size);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_init()
{
    // 初始化串口 (中断模式)
    g_uart = SerialInit(UART_0_INST, SERIAL_MODE_IT, 
                        error_callback, NULL);

    // 创建任务
    xTaskCreate(uart_task, "UART", 256, NULL, 
                tskIDLE_PRIORITY + 1, NULL);
}

// 在 UART ISR 中
void UART0_IRQHandler(void)
{
    SerialIRQ(g_uart);
}
```

---

## SysConfig 配置建议

为了正确使用此库，建议在 SysConfig 中进行以下配置：

1. **UART 外设**：
   - 启用 RX FIFO
   - 启用中断：`DL_UART_INTERRUPT_RX`
   - (可选) 启用空闲中断

2. **DMA (如使用 DMA 模式)**：
   - 配置 DMA 通道 0 为接收通道
   - 设置地址模式、数据宽度等

3. **中断优先级**：
   - UART 中断优先级建议为 5-7

---

## 注意事项

1. **线程安全**：建议在单一任务中使用此库，或在多任务环境中添加互斥锁
2. **缓冲区大小**：默认 TX 256 字节，RX 512 字节，可在 `SerialInit` 中修改
3. **超时单位**：所有超时参数单位为毫秒 (ms)
4. **内存占用**：FreeRTOS 环境下每个句柄约占 900+ 字节

---

## 常见问题 (FAQ)

**Q: 如何在 DMA 模式下工作？**
A: 在应用层手动配置 DMA，库会自动处理 DMA 完成中断。参考 `uart_rx_multibyte_fifo_dma_interrupts.c`。

**Q: 支持多个 UART 吗？**
A: 支持，为每个 UART 创建独立的 `SerialHandle_t` 即可。

**Q: 如何切换工作模式？**
A: 反初始化后重新初始化：
```c
SerialDeinit(handle);
handle = SerialInit(hw_uart, NEW_MODE, cb, param);
```

---

## 版本历史

- **v1.0** (2026-05-21)：初始发布
  - 支持 POLL/IT/DMA 三种模式
  - FreeRTOS 集成
  - 完整的错误处理
