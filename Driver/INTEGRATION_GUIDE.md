# 串口库集成指南

## 快速开始

### 第1步：添加文件到项目

将以下文件添加到 `empty_freertos/Driver/` 目录：
- `uart.h` - 头文件（已存在）
- `uart.c` - 实现文件（已存在）

### 第2步：配置 SysConfig

在 `empty.syscfg` 中确保以下配置：

```javascript
UART1.$name = "UART_0";
UART1.profile = "CONFIG_PROFILE_1";
UART1.peripheral.$assign = "UART0";
UART1.targetBaudRate = 9600;
UART1.enableInternalLoopback = false;
UART1.enableFIFO = true;

// 根据工作模式选择配置：

// 如果使用中断模式：
UART1.enabledInterrupts = ["RX"];  // 启用接收中断

// 如果使用 DMA 模式：
UART1.enabledInterrupts = ["DMA_DONE_RX"];
UART1.enableFIFO = true;
UART1.enabledDMARXTriggers = "DL_UART_DMA_INTERRUPT_RX";
UART1.enableDMARX = true;
UART1.DMA_CHANNEL_RX.peripheral.$assign = "DMA_CH0";
UART1.DMA_CHANNEL_RX.addressMode = "f2b";
UART1.DMA_CHANNEL_RX.dstIncDec = "INCREMENT";
```

### 第3步：修改 UART ISR

在生成的 `ti_msp_dl_config.c` 中找到 `UART0_IRQHandler`，修改为：

```c
// 声明全局串口句柄（在应用代码中）
extern SerialHandle_t *g_uart_handle;

void UART0_IRQHandler(void)
{
    if (g_uart_handle) {
        SerialIRQ(g_uart_handle);
    }
}
```

### 第4步：在应用代码中初始化

```c
#include "Driver/uart.h"
#include <FreeRTOS.h>
#include <task.h>

// 全局句柄
SerialHandle_t *g_uart_handle = NULL;

void error_handler(int err_code, void *param)
{
    // 处理错误
}

void recv_handler(uint8_t *data, uint16_t size, void *param)
{
    // 处理接收数据
}

void app_uart_init(void)
{
    // 初始化串口（中断模式）
    g_uart_handle = SerialInit(
        UART_0_INST,           // UART 硬件实例
        SERIAL_MODE_IT,        // 中断模式
        error_handler,         // 错误回调
        NULL                   // 用户参数
    );

    if (!g_uart_handle) {
        // 初始化失败处理
        while (1);
    }
}
```

---

## 三种工作模式对比

### 1. 轮询模式 (SERIAL_MODE_POLL)

**初始化**：
```c
g_uart_handle = SerialInit(UART_0_INST, SERIAL_MODE_POLL, NULL, NULL);
```

**优点**：
- 无需中断配置
- 实现简单
- 无需 FreeRTOS

**缺点**：
- 效率低
- CPU 占用高

**应用场景**：
- 调试
- 低速通信
- 不需要中断的简单应用

---

### 2. 中断模式 (SERIAL_MODE_IT)

**初始化**：
```c
g_uart_handle = SerialInit(UART_0_INST, SERIAL_MODE_IT, error_handler, NULL);
```

**关键步骤**：
1. SysConfig 中启用 `RX` 中断
2. 修改 UART ISR，调用 `SerialIRQ()`
3. 创建 FreeRTOS 任务处理接收数据

**示例任务**：
```c
void uart_task(void *param)
{
    uint8_t buffer[64];
    int size;

    while (1) {
        // 等待接收不定长数据（最多 500ms）
        size = SerialReceiveIDLE(g_uart_handle, buffer, sizeof(buffer), 500);

        if (size > 0) {
            // 处理接收到的数据
            printf("Received: %d bytes\n", size);
            
            // 回复
            SerialTransmit(g_uart_handle, buffer, size);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

**优点**：
- 效率高
- 实时性好
- CPU 占用低

**缺点**：
- 需要中断配置
- 需要 FreeRTOS

**应用场景**：
- 实时通信
- 需要低功耗
- 高速数据传输

---

### 3. DMA 模式 (SERIAL_MODE_DMA)

**初始化**：
```c
g_uart_handle = SerialInit(UART_0_INST, SERIAL_MODE_DMA, NULL, NULL);
```

**SysConfig 配置**：
```javascript
UART1.enabledInterrupts = ["DMA_DONE_RX"];
UART1.enableFIFO = true;
UART1.enabledDMARXTriggers = "DL_UART_DMA_INTERRUPT_RX";
UART1.enableDMARX = true;
UART1.DMA_CHANNEL_RX.peripheral.$assign = "DMA_CH0";
```

**应用层配置**：
```c
#include <ti/driverlib/dl_dma.h>

void setup_dma_rx(SerialHandle_t *handle, uint8_t *buffer, uint16_t size)
{
    // 设置 DMA 源地址（UART RX FIFO）
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, 
                      (uint32_t)(&UART_0_INST->RXDATA));
    
    // 设置 DMA 目的地址
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)buffer);
    
    // 设置传输大小
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, size);
    
    // 启用 DMA 通道
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}
```

**优点**：
- 最高效率
- CPU 负载最低
- 适合大数据量传输

**缺点**：
- 配置复杂
- 需要 SysConfig 和应用层双重配置
- 调试困难

**应用场景**：
- 大数据量传输
- 低功耗应用
- 高速数据流

---

## 代码示例

### 示例1：简单的命令应答系统

```c
#include "uart.h"

SerialHandle_t *g_uart = NULL;

void uart_task(void *param)
{
    uint8_t cmd[64];
    uint8_t response[64];
    int size;

    while (1) {
        // 接收命令（不定长，以 '\n' 结尾）
        size = SerialReceiveIDLE(g_uart, cmd, sizeof(cmd), 1000);

        if (size > 0) {
            // 处理命令
            if (cmd[0] == 'L') {
                // LED 命令
                sprintf((char*)response, "LED Ctrl OK\n");
            } else if (cmd[0] == 'T') {
                // 温度查询
                sprintf((char*)response, "Temp: 25C\n");
            } else {
                sprintf((char*)response, "Unknown\n");
            }

            // 发送响应
            SerialTransmit(g_uart, response, strlen((char*)response));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_init(void)
{
    g_uart = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);
    xTaskCreate(uart_task, "UART_CMD", 256, NULL, 
                tskIDLE_PRIORITY + 1, NULL);
}
```

### 示例2：接收定长数据包

```c
void uart_task(void *param)
{
    uint8_t packet[4];  // 接收 4 字节数据包

    while (1) {
        // 接收定长数据（4 字节，超时 500ms）
        int size = SerialReceive(g_uart, packet, 4, 500);

        if (size == 4) {
            // 处理数据包
            uint32_t value = (packet[0] << 24) | (packet[1] << 16) |
                            (packet[2] << 8) | packet[3];
            printf("Received: 0x%08X\n", value);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### 示例3：错误处理

```c
void my_error_callback(int err_code, void *param)
{
    switch (err_code) {
        case SERIAL_ERR_TIMEOUT:
            // 日志记录或重试
            printf("UART Rx Timeout\n");
            break;

        case SERIAL_ERR_BUSY:
            // 缓冲区满，丢弃数据
            printf("UART Buffer Full - Data Lost\n");
            break;

        case SERIAL_ERR_INVALID:
            printf("Invalid UART Handle\n");
            break;

        default:
            printf("Unknown UART Error: %d\n", err_code);
            break;
    }
}

// 初始化时注册回调
g_uart = SerialInit(UART_0_INST, SERIAL_MODE_IT, 
                    my_error_callback, NULL);
```

---

## 故障排除

### 问题1：收不到数据

**检查清单**：
1. ✓ SysConfig 中 UART 启用了 RX 中断
2. ✓ UART ISR 中调用了 `SerialIRQ()`
3. ✓ 波特率设置正确
4. ✓ 接收超时设置合理

**解决方案**：
```c
// 添加调试信息
void debug_uart_status(SerialHandle_t *h)
{
    printf("UART Mode: %d\n", h->mode);
    printf("Rx Buffer: %d bytes\n", h->recv_count);
    printf("Rx Done: %s\n", h->rx_done ? "Yes" : "No");
}
```

### 问题2：数据损坏

**可能原因**：
- 波特率不匹配
- 缓冲区太小
- 中断延迟

**解决方案**：
```c
// 增加缓冲区大小
SerialHandle_t *handle = SerialInit(...);
// 重新分配较大的缓冲区
serial_free(handle->recv_buffer);
handle->recv_buffer_size = 1024;
handle->recv_buffer = serial_malloc(1024);
```

### 问题3：任务卡死

**原因**：通常是 `SerialReceive()` 超时等待

**解决方案**：
```c
// 使用非阻塞超时
int size = SerialReceive(g_uart, buffer, 64, 100);  // 100ms 超时
if (size == SERIAL_ERR_TIMEOUT) {
    // 处理超时
    vTaskDelay(pdMS_TO_TICKS(10));  // 让出 CPU
}
```

---

## 性能指标

| 指标 | 轮询 | 中断 | DMA |
|-----|------|------|-----|
| 吞吐量 | ~100 bps | ~10 kbps | ~100+ kbps |
| CPU 占用 | 高 | 低 | 最低 |
| 延迟 | 高 | 低 | 最低 |
| 配置复杂度 | 简单 | 中等 | 复杂 |
| 适合场景 | 调试 | 实时 | 高速 |

---

## 下一步

1. 参考 `uart_example.c` 了解完整用法
2. 阅读 `UART_README.md` 获得详细 API 说明
3. 根据场景选择合适的工作模式
4. 在项目中集成和测试

祝您使用愉快！🎉
