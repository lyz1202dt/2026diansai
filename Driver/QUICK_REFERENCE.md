# UART 库快速参考

## 🚀 30 秒快速开始

```c
#include "uart.h"

// 1. 声明全局句柄
SerialHandle_t *h = NULL;

// 2. 初始化
h = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);

// 3. 发送
SerialTransmit(h, (uint8_t*)"Hello\n", 6);

// 4. 接收
uint8_t buf[64];
int sz = SerialReceive(h, buf, 64, 1000);

// 5. ISR
void UART0_IRQHandler(void) { SerialIRQ(h); }
```

---

## 📖 常用 API

### 初始化
```c
SerialHandle_t* SerialInit(
    UART_Regs *hw,              // UART_0_INST
    uint8_t mode,               // SERIAL_MODE_IT/POLL/DMA
    ErrorCb error_cb,           // 错误回调或 NULL
    void *param                 // 用户参数或 NULL
);
```

### 发送
```c
int SerialTransmit(
    SerialHandle_t *h,          // 句柄
    uint8_t *data,              // 数据指针
    uint16_t size               // 数据长度
);
// 返回: 发送字节数 或 错误码
```

### 定长接收
```c
int SerialReceive(
    SerialHandle_t *h,          // 句柄
    uint8_t *data,              // 接收缓冲
    uint16_t size,              // 接收长度
    int timeout                 // 超时(ms)
);
// 返回: 实际接收字节数 或 错误码
```

### 不定长接收
```c
int SerialReceiveIDLE(
    SerialHandle_t *h,          // 句柄
    uint8_t *data,              // 接收缓冲
    uint16_t max_size,          // 最大长度
    int timeout                 // 超时(ms)
);
// 返回: 实际接收字节数 或 错误码
```

---

## 🎯 工作模式选择

| 需求 | 推荐 | 配置 |
|------|------|------|
| 调试 | POLL | 无中断 |
| 实时 | IT | RX 中断 |
| 高速 | DMA | DMA 中断 |

---

## ⚠️ 错误码

```c
SERIAL_OK           (0)   // 成功
SERIAL_ERR_TIMEOUT  (-1)  // 超时
SERIAL_ERR_INVALID  (-2)  // 无效参数
SERIAL_ERR_BUSY     (-3)  // 缓冲区满
SERIAL_ERR_ALLOC    (-4)  // 内存不足
```

---

## 🔧 关键实现

### 中断处理
```c
void UART0_IRQHandler(void)
{
    if (g_uart_handle) {
        SerialIRQ(g_uart_handle);
    }
}
```

### FreeRTOS 任务
```c
void uart_task(void *p)
{
    uint8_t buf[64];
    
    while (1) {
        int sz = SerialReceiveIDLE(g_uart, buf, 64, 500);
        if (sz > 0) {
            // 处理数据
            SerialTransmit(g_uart, buf, sz);
        }
    }
}
```

### 错误处理
```c
void error_cb(int err, void *p)
{
    if (err == SERIAL_ERR_TIMEOUT) {
        // 处理超时
    } else if (err == SERIAL_ERR_BUSY) {
        // 处理缓冲区满
    }
}
```

---

## 💾 内存占用

```
代码: ~5 KB
数据: ~800 B
缓冲: ~768 B (256+512)
---
总计: ~6.5 KB
```

---

## 🧪 快速测试

### 轮询测试
```c
// 初始化为轮询模式
h = SerialInit(UART_0_INST, SERIAL_MODE_POLL, NULL, NULL);

// 发送并接收
SerialTransmit(h, (uint8_t*)"test\n", 5);
uint8_t echo[64];
int sz = SerialReceive(h, echo, 64, 1000);
```

### 中断测试
```c
// 初始化为中断模式
h = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);

// 在任务中接收
int sz = SerialReceive(h, buf, 64, 500);
```

---

## 📋 SysConfig 配置清单

- [ ] UART 使能
- [ ] 波特率设置 (推荐 9600/115200)
- [ ] RX/TX 引脚分配
- [ ] FIFO 使能
- [ ] 中断使能 (RX 或 DMA_DONE_RX)
- [ ] (可选) DMA 通道配置

---

## 🔗 相关文件

- `uart.h` - 头文件 (API 定义)
- `uart.c` - 实现文件
- `uart_example.c` - 使用示例
- `UART_README.md` - 详细文档
- `INTEGRATION_GUIDE.md` - 集成指南

---

## 💡 提示

1. **超时设置** 建议：
   - 快速响应: 50-100 ms
   - 一般应用: 500-1000 ms
   - 无限等待: -1

2. **缓冲区大小** 建议：
   - 最小: 32 字节
   - 常规: 256 字节
   - 大数据: 1024+ 字节

3. **中断优先级** 建议：
   - 相对较高 (4-6)
   - 不要最高 (0-3)
   - 不要最低 (12-15)

4. **调试技巧**：
   ```c
   // 添加调试打印
   printf("UART Rx: %d bytes, Status: %d\n", 
          h->recv_count, h->rx_done);
   ```

---

**快速参考 v1.0** | 更多信息见文档
